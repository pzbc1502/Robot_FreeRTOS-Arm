#include "robot-take.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_uart.h"
#include "K230_cmd.h"
#include "gripper.h"
#include "bsp_led.h"

/* 说明：
 * - 0x02：无草莓时用于触发 Z 轴上移/搜索。
 * - 0x03：草莓坐标偏差，用于对准（画面 x,y -> 机械臂 x,z）。
 * - 0x04：对准后的夹取门控：0=允许接近，1=到夹取点。
 * - 0x06：用于草莓的计数：成熟草莓，未成熟草莓。
 * - 该状态机由 `vision_service_thread_entry.c` 周期调用 `robot_take_step()` 驱动。
 * - 运动使用 `robot_send_auto_event()`（基座坐标绝对位置）。
 */

    /* 采摘主流程状态机：
     * - 所有运动命令最终走 `robot_send_auto_event()`，这里仅做状态切换与参数更新。
     * - 0x02/0x03/0x04 由视觉线程注入到 `robot_take_step()` 的 `obs` 中。
     */
    typedef enum {
        TAKE_INIT = 0,          //初始化/复位状态：回到预备位 pre，打开补光灯，夹爪复位
        TAKE_WAIT_TARGET,       //等待目标：收到 0x03 坐标→对准；0x02=0→进入 Z 搜索
        TAKE_SEARCH_Z,          //Z 轴搜索：在安全范围内上下扫描 Z，直到重新收到目标坐标
        TAKE_ALIGN,             //对准：根据 0x03 的像素偏差，微调 x/z，直到对准阈值
        TAKE_WAIT_GRASP,        //等待夹取许可：根据 0x04 决定是接近还是直接夹取
        TAKE_APPROACH,          //接近：沿 -Y 小步逼近草莓，等待 0x04=1
        TAKE_GRASP,             //夹取：执行夹爪力控/电流阈值夹取
        TAKE_PLUCK_DOWN,        //摘取：向下(-Z)微动作并保持一段时间
        TAKE_LIFT,              //抬起：回到安全高度 Z_SAFE
        TAKE_PLACE,             //放置：去箱子 → 开夹 → 回预备位
        TAKE_RECOVER,           //恢复：异常/超时回安全位，再回预备位
    } take_state_t;


typedef struct {
    take_state_t st;
    uint32_t enter_ms;

    struct position pre;
    struct position box;

    struct position target;
    bool target_active;

    /* K230 输入缓存 */
    bool has_flag02;
    uint8_t flag02;
    uint32_t flag02_ms;

    bool has_flag04;
    uint8_t flag04;
    bool flag04_latched;
    uint32_t flag04_ms;

    bool has_coord;
    int16_t dcx;
    int16_t dcy;

    /* Z扫查 */
    float scan_z;
    float scan_dir; /* +1 or -1 */

    /* 接近参数 */
    uint32_t approach_steps;
    uint32_t last_step_ms;

    /* 放置/恢复 子阶段 */
    uint8_t place_stage;
    uint8_t recover_stage;

    /* 向下摘取计时 */
    uint32_t pluck_reached_ms;

    /* 非阻塞稳态计时/单次动作标志 */
    uint32_t settle_ms;
    bool place_open_done;
    bool place_cmd_sent;
    uint8_t k230_prestart_sent;
} take_ctx_t;

static take_ctx_t s;

volatile bool ROBOT_TAKE_ENABLED = false;
volatile uint32_t g_strawberry_picked_count = 0;

static const char *take_state_name(take_state_t st)
{
    switch (st) {
        case TAKE_INIT:       return "TAKE_INIT";
        case TAKE_WAIT_TARGET:return "TAKE_WAIT_TARGET";
        case TAKE_SEARCH_Z:   return "TAKE_SEARCH_Z";
        case TAKE_ALIGN:      return "TAKE_ALIGN";
        case TAKE_WAIT_GRASP: return "TAKE_WAIT_GRASP";
        case TAKE_APPROACH:   return "TAKE_APPROACH";
        case TAKE_GRASP:      return "TAKE_GRASP";
        case TAKE_PLUCK_DOWN: return "TAKE_PLUCK_DOWN";
        case TAKE_LIFT:       return "TAKE_LIFT";
        case TAKE_PLACE:      return "TAKE_PLACE";
        case TAKE_RECOVER:    return "TAKE_RECOVER";
        default:              return "TAKE_UNKNOWN";
    }
}

static inline void enter_state(take_state_t st, uint32_t now)
{
    take_state_t prev = s.st;
    s.st = st;              // 切换状态
    s.enter_ms = now;       // 记录进入时间（用于超时判断）
    s.target_active = false;// 清除“有未完成的运动指令”标志
    s.settle_ms = 0u;       // 清除稳态计时
    s.place_open_done = false;
    s.place_cmd_sent = false;
    s.k230_prestart_sent = 0u;
    if ((st == TAKE_INIT) || (st == TAKE_WAIT_TARGET) || (st == TAKE_RECOVER)) {
        s.flag04_latched = false;
    }
    LOG("robot_take: state %s -> %s\r\n", take_state_name(prev), take_state_name(st));
}


static inline bool pos_near(const struct position *a, const struct position *b, float tol)
{
    return (fabsf(a->x - b->x) <= tol) && (fabsf(a->y - b->y) <= tol) && (fabsf(a->z - b->z) <= tol);
}

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float calc_place_j1_rel_delta(void)
{
    float sign = (ROBOT_TAKE_BOX_BASE_DIR == 0u) ? 1.0f : -1.0f;
    return sign * ROBOT_TAKE_BOX_BASE_ROTATE_DEG;
}

static bool send_auto(const struct position *p)
{
    if (!p) return false;
    return (robot_send_auto_event((struct position *)p) != 0);
}

static bool coord_aligned(void)
{
    // 判断像素偏差是否进入允许阈值（越小越准）
    int16_t ax = s.dcx;
    int16_t ay = s.dcy;
    if (ax < 0) ax = (int16_t)-ax;
    if (ay < 0) ay = (int16_t)-ay;
    return (ax <= ROBOT_TAKE_ALIGN_TOL_PX) && (ay <= ROBOT_TAKE_ALIGN_TOL_PX);
}

static inline bool flag04_grasp_ready(void)
{
    return (s.flag04_latched || (s.has_flag04 && (s.flag04 == 1u)));
}

static void update_inputs(const robot_take_obs_t *obs)
{
    /* 时间基准说明：
     * - 优先使用 `obs->now_ms`：由视觉线程注入统一时间，便于日志对齐。
     * - 未提供时回退到 `HAL_GetTick()`。
     */
    uint32_t now = (obs && obs->now_ms) ? obs->now_ms : HAL_GetTick();

    if (obs)
    {
        // 0x03 坐标（像素偏差）
        s.has_coord = obs->has_coord;
        if (obs->has_coord) {
            s.dcx = obs->dcx;   // 画面 x 偏差
            s.dcy = obs->dcy;   // 画面 y 偏差
        }

        // 0x02：草莓完整性/存在标志
        if (obs->has_flag02) {
            s.has_flag02 = true;
            s.flag02 = obs->flag02;
            s.flag02_ms = now;
        }

        // 0x04：夹取许可标志
        if (obs->has_flag04) {
            s.has_flag04 = true;
            s.flag04 = obs->flag04;
            s.flag04_ms = now;
            if (obs->flag04 == 1u) {
                s.flag04_latched = true;
            }
        }
    }

    // 超时后清除标志，避免使用陈旧视觉数据
    if (s.has_flag02 && ((now - s.flag02_ms) > ROBOT_TAKE_K230_FLAG02_VALID_MS)) {
        s.has_flag02 = false;
    }

    if (s.has_flag04 && ((now - s.flag04_ms) > ROBOT_TAKE_K230_FLAG_VALID_MS)) {
        s.has_flag04 = false;
    }
}


static void apply_xz_trim(struct position *p)
{
    if (!p) return;
    if (!s.has_coord) return;

    // 像素偏差 -> 机械臂 x/z 微调（mm）
    float dx = (float)s.dcx * ROBOT_TAKE_KX_MM_PER_PX;
    float dz = (float)s.dcy * ROBOT_TAKE_KY_MM_PER_PX;

    // 每一步最大修正，避免一次跳太大
    dx = clampf(dx, -ROBOT_TAKE_XY_MAX_STEP_MM, ROBOT_TAKE_XY_MAX_STEP_MM);
    dz = clampf(dz, -ROBOT_TAKE_XY_MAX_STEP_MM, ROBOT_TAKE_XY_MAX_STEP_MM);

    p->x += dx;
    p->z += dz;

    // Z 目标下限以上抬升扫描最低点为基准，避免出现 z<ROBOT_TAKE_Z_MIN 导致逆解失败
    p->z = clampf(p->z, ROBOT_TAKE_Z_MIN, ROBOT_TAKE_Z_MAX);
}



void robot_take_init(void)
{
    memset(&s, 0, sizeof(s));

	//配置预开始位置
    s.pre.x = ROBOT_TAKE_PRE_X;
    s.pre.y = ROBOT_TAKE_PRE_Y;
    s.pre.z = ROBOT_TAKE_Z_SAFE;

	//配置箱子位置
    s.box.x = ROBOT_TAKE_BOX_X;
    s.box.y = ROBOT_TAKE_BOX_Y;
    s.box.z = ROBOT_TAKE_BOX_Z;

	//配置扫描的Z最低点
    s.scan_z = ROBOT_TAKE_Z_MIN;
	
	//配置扫描方向   1 或 -1
    s.scan_dir = 1.0f;

	//夹爪初始化
    gripper_init();
	
	// 默认关闭补光灯，等待流程启动再开启
    K230_LED_OFF; 

	//开启状态机
    enter_state(TAKE_INIT, HAL_GetTick());
    LOG("robot_take: init done (DISABLED by default).\r\n");
    LOG("Set 'ROBOT_TAKE_ENABLED = true' in Watch window to start.\r\n");

}


//机械臂夹取主任务状态机
void robot_take_step(const robot_take_obs_t *obs)
{
    if (!ROBOT_TAKE_ENABLED) {
        return;
    }


    /* 同 `update_inputs()`：优先用外部注入的时间戳，避免状态机自己去依赖/管理 FSP 定时器。 */
    uint32_t now = (obs && obs->now_ms) ? obs->now_ms : HAL_GetTick();

    update_inputs(obs);

    switch (s.st)
    {
        case TAKE_INIT:        // 初始：回到预备位
        {
            if (!s.target_active)
            {
                K230_LED_ON; // 进入流程时开补光灯
                (void) gripper_stop(6);
                (void) gripper_open(); // 夹爪复位为张开

                s.target = s.pre;       // 目标位 = 预备位

                s.target_active = send_auto(&s.target);

                s.approach_steps = 0;   // 清零接近步数
                s.last_step_ms = now;   // 记录时间戳

                LOG("robot_take: go PRE (%.1f,%.1f,%.1f)\r\n", s.pre.x, s.pre.y, s.pre.z);
            }


            if (s.target_active && pos_near(&g_robot.cur_pos, &s.pre, 2.0f))
            {
                (void) k230_send_status_u8(RA6_FUNC_AT_PRESTART, 0x01u); // 告知视觉已到预备位
                enter_state(TAKE_WAIT_TARGET, now);
                LOG("robot_take: at PRE -> wait target\r\n");
            }
            break;
        }


        case TAKE_WAIT_TARGET: // 等待目标/坐标

        {
			if (s.has_flag02 && (s.flag02 == 0u))
            {
                // 0x02=0：画面无草莓，开始 Z 扫描
                enter_state(TAKE_SEARCH_Z, now);
                s.scan_z = ROBOT_TAKE_Z_MIN;
                s.scan_dir = 1.0f;
                s.target_active = false;
                LOG("robot_take: no strawberry -> search Z\r\n");
            }
			
			
			//0x03，收到草莓坐标像素差值，机械臂移动，让草莓在画面的中心
            if (s.has_coord)
            {
                // 收到 0x03 坐标，进入对准流程
                enter_state(TAKE_ALIGN, now);
                s.last_step_ms = now;
                s.target_active = false;
                LOG("robot_take: got coord -> align\r\n");
                break;
            }

            break;
        }


        case TAKE_SEARCH_Z:    // 搜索Z方向

        {
            if (s.has_coord)
            {
                // 一旦重新看到目标坐标，立刻转入对准
                enter_state(TAKE_ALIGN, now);
                s.last_step_ms = now;
                s.target_active = false;
                LOG("robot_take: coord arrived -> align\r\n");
                break;
            }

            if (!s.target_active)
            {
                // 在安全范围内上下扫描 Z
                float z = s.scan_z;
                if (z < ROBOT_TAKE_Z_MIN) z = ROBOT_TAKE_Z_MIN;
                if (z > ROBOT_TAKE_Z_MAX) z = ROBOT_TAKE_Z_MAX;

                s.target = s.pre;
                s.target.z = z;
                s.target_active = send_auto(&s.target);

                // 下一次扫描位置
                s.scan_z += s.scan_dir * ROBOT_TAKE_SCAN_DZ;
                if (s.scan_z >= ROBOT_TAKE_Z_MAX) { s.scan_z = ROBOT_TAKE_Z_MAX; s.scan_dir = -1.0f; }
                if (s.scan_z <= ROBOT_TAKE_Z_MIN) { s.scan_z = ROBOT_TAKE_Z_MIN; s.scan_dir =  1.0f; }
            }
            else
            {
                if (pos_near(&g_robot.cur_pos, &s.target, 2.0f)) {
                    // 一个扫描点到位后，继续在 SEARCH_Z 内发下一步，实现连续往返扫描
                    s.target_active = false;
                    LOG("robot_take: search step done -> continue search\r\n");
                }
            }
            break;
        }


        case TAKE_ALIGN:       // 对准目标

        {
            if (flag04_grasp_ready())
            {
                s.flag04_latched = false;
                enter_state(TAKE_GRASP, now);
                LOG("robot_take: align got flag04=1(latched/realtime) -> grasp\r\n");
                break;
            }

            // 0x04=0 优先触发逼近，避免持续坐标对准占用 04 的步进意图
            if (s.has_flag04 && (s.flag04 == 0u))
            {
                enter_state(TAKE_APPROACH, now);
                s.last_step_ms = now;
                LOG("robot_take: align got flag04=0 -> approach\r\n");
                break;
            }

            if (!s.has_coord)
            {
                // 坐标丢失，回到等待
                enter_state(TAKE_WAIT_TARGET, now);
                LOG("robot_take: coord lost -> wait target\r\n");
                break;
            }

            if (coord_aligned())
            {
                // 像素偏差满足阈值，进入等待夹取许可
                enter_state(TAKE_WAIT_GRASP, now);
                LOG("robot_take: aligned -> wait 0x04\r\n");
                break;
            }

            if (!s.target_active)
            {
                // 到达步进周期再做一次微调
                if ((now - s.last_step_ms) < ROBOT_TAKE_ALIGN_STEP_PERIOD_MS) {
                    break;
                }

                s.target = g_robot.cur_pos;
                apply_xz_trim(&s.target);   // 将像素偏差换算为 x/z 微调
                s.target_active = send_auto(&s.target);
                s.last_step_ms = now;

                LOG("robot_take: align step -> (%.1f,%.1f,%.1f)\r\n", s.target.x, s.target.y, s.target.z);
            }
            else
            {
                if (pos_near(&g_robot.cur_pos, &s.target, 2.0f)) {
                    s.target_active = false; // 一次微调完成，等待下一次
                }
            }
            break;
        }


        case TAKE_WAIT_GRASP:			//等待 0x04  01 夹取许可
        {
            if (flag04_grasp_ready())
            {
                s.flag04_latched = false;
                enter_state(TAKE_GRASP, now);
                LOG("robot_take: flag04=1(latched/realtime) -> grasp\r\n");
                break;
            }

            if (s.has_flag04 && (s.flag04 == 0u))
            {
                enter_state(TAKE_APPROACH, now);
                s.last_step_ms = now;
                LOG("robot_take: flag04=0 -> approach\r\n");
                break;
            }

            if ((now - s.enter_ms) > ROBOT_TAKE_WAIT_ALIGN_TIMEOUT_MS)
            {
                enter_state(TAKE_WAIT_TARGET, now);
                LOG("robot_take: wait 0x04 timeout -> wait target\r\n");
            }
            break;
        }

		
        case TAKE_APPROACH:			//基座坐标 -Y 方向接近（步长由宏控制）
        {
            if (flag04_grasp_ready()) {
                s.flag04_latched = false;
                enter_state(TAKE_GRASP, now);
                LOG("robot_take: approach got flag04=1(latched/realtime) -> grasp\r\n");
                break;
            }

            if (s.approach_steps >= ROBOT_TAKE_MAX_APPROACH_STEPS) {
                enter_state(TAKE_RECOVER, now);
                s.recover_stage = 0;
                LOG("robot_take: approach max -> recover\r\n");
                break;
            }

            if (!s.target_active)
            {
                if ((now - s.last_step_ms) < ROBOT_TAKE_APPROACH_STEP_PERIOD_MS) {
                    break;
                }

                s.target = g_robot.cur_pos;
                s.target.y -= ROBOT_TAKE_APPROACH_DY;
                s.target_active = send_auto(&s.target);
                s.last_step_ms = now;
                s.approach_steps++;

                LOG("robot_take: step %lu -> (%.1f,%.1f,%.1f)\r\n",
                    (unsigned long)s.approach_steps, s.target.x, s.target.y, s.target.z);
            }
            else
            {
                if (pos_near(&g_robot.cur_pos, &s.target, 2.0f)) {
                    s.target_active = false;
                    enter_state(TAKE_WAIT_GRASP, now);
                }
            }

            if (!s.has_flag04 && ((now - s.enter_ms) > ROBOT_TAKE_K230_FLAG_VALID_MS)) {
                enter_state(TAKE_WAIT_TARGET, now);
                LOG("robot_take: approach flag04 lost -> wait target\r\n");
            }
            break;
        }

        case TAKE_GRASP:			//执行夹爪夹取（力控/电流阈值）
        {
            gripper_fruit_t fruit = GRIPPER_FRUIT_STRAWBERRY;
            gripper_grasp_cfg_t cfg = {0};
            (void) gripper_get_preset(fruit, &cfg);


            uint16_t touch = 0u;
            gripper_result_t r = gripper_grasp_force(&cfg, &touch);
            if (r == GRIPPER_OK) {
                enter_state(TAKE_PLUCK_DOWN, now);
                s.pluck_reached_ms = 0u;
                LOG("robot_take: grasp ok touch=%u\r\n", (unsigned)touch);
            } else {
                enter_state(TAKE_RECOVER, now);
                s.recover_stage = 0;
                LOG("robot_take: grasp fail r=%d\r\n", (int)r);
            }
            break;
        }

        case TAKE_PLUCK_DOWN:		//摘取微动作：向下(-Z)
        {
            if (!s.target_active)
            {
                s.target = g_robot.cur_pos;
                s.target.z -= ROBOT_TAKE_PLUCK_DZ;
                s.target.z = clampf(s.target.z, ROBOT_TAKE_Z_MIN, ROBOT_TAKE_Z_MAX);
                s.target_active = send_auto(&s.target);
                LOG("robot_take: pluck down\r\n");
            }

            if (s.target_active && pos_near(&g_robot.cur_pos, &s.target, 2.0f))
            {
                if (s.pluck_reached_ms == 0u) {
                    s.pluck_reached_ms = now;
                }
                if ((now - s.pluck_reached_ms) >= ROBOT_TAKE_PLUCK_HOLD_MS) {
                    enter_state(TAKE_LIFT, now);
                    LOG("robot_take: pluck done -> lift\r\n");
                }
            }
            break;
        }

        case TAKE_LIFT:			//抬起/撤离
        {
            if (!s.target_active)
            {
                s.target = g_robot.cur_pos;
                s.target.z = ROBOT_TAKE_Z_SAFE;
                s.target_active = send_auto(&s.target);
                s.settle_ms = 0u;
                LOG("robot_take: lift\r\n");
            }

            if (s.target_active)
            {
                if (pos_near(&g_robot.cur_pos, &s.target, 2.0f))
                {
                    if (s.settle_ms == 0u) {
                        s.settle_ms = now;
                    }
                    if ((now - s.settle_ms) >= ROBOT_TAKE_LIFT_SETTLE_MS)
                    {
                        enter_state(TAKE_PLACE, now);
                        s.place_stage = 0u;
                        LOG("robot_take: lift ok -> place\r\n");
                    }
                }
                else
                {
                    s.settle_ms = 0u;
                }
            }
            break;
        }

		
        case TAKE_PLACE:			//放置闭环：去箱子 -> J1旋转 -> 开夹 -> J1回home -> soft/hard reset -> 回TAKE_INIT
        {
            /*
             * stage0 : go box
             * stage1 : send J1 relative rotate to place (+/-deg)
             * stage2 : wait rotate settle
             * stage3 : open gripper + hold
             * stage4 : send J1 rotate back home
             * stage5 : wait rotate settle
             * stage6 : send soft reset
             * stage7 : wait soft reset done (fixed time)
             * stage8 : send hard reset
             * stage9 : wait hard reset done (fixed time)
             * stage10: send K230 prestart status N times
             */
            if (s.place_stage == 0u)
            {
                if (!s.target_active) {
                    s.target = s.box;
                    s.target_active = send_auto(&s.target);
                    s.settle_ms = 0u;
                    LOG("robot_take: go box\r\n");
                }

                if (s.target_active)
                {
                    if (pos_near(&g_robot.cur_pos, &s.target, 2.0f))
                    {
                        if (s.settle_ms == 0u) {
                            s.settle_ms = now;
                        }
                        if ((now - s.settle_ms) >= ROBOT_TAKE_PLACE_SETTLE_MS) {
                            s.target_active = false;
                            s.place_stage = 1u;
                            s.settle_ms = 0u;
                            s.place_cmd_sent = false;
                        }
                    }
                    else
                    {
                        s.settle_ms = 0u;
                    }
                }
            }
            else if (s.place_stage == 1u)
            {
                if (!s.place_cmd_sent)
                {
                    float rel_deg = calc_place_j1_rel_delta();
                    s.place_cmd_sent = (robot_send_rel_rotate_event(ROBOT_JOINT_1, rel_deg) != 0);
                    s.settle_ms = now;
                    LOG("robot_take: rel rotate J1 to place delta=%.2f\r\n", rel_deg);
                }

                if (s.place_cmd_sent)
                {
                    s.place_stage = 2u;
                    s.place_cmd_sent = false;
                    s.settle_ms = now;
                }
            }
            else if (s.place_stage == 2u)
            {
                if ((now - s.settle_ms) >= ROBOT_TAKE_BASE_ROTATE_SETTLE_MS)
                {
                    s.place_stage = 3u;
                    s.settle_ms = 0u;
                }
            }
            else if (s.place_stage == 3u)
            {
                if (!s.place_open_done)
                {
                    (void) gripper_open();
                    s.place_open_done = true;
                    s.settle_ms = now;
                }

                if ((now - s.settle_ms) >= ROBOT_TAKE_GRIPPER_OPEN_HOLD_MS)
                {
                    s.place_stage = 4u;
                    s.place_open_done = false;
                    s.settle_ms = 0u;
                }
            }
            else if (s.place_stage == 4u)
            {
                if (!s.place_cmd_sent)
                {
                    float rel_back = -calc_place_j1_rel_delta();
                    s.place_cmd_sent = (robot_send_rel_rotate_event(ROBOT_JOINT_1, rel_back) != 0);
                    s.settle_ms = now;
                    LOG("robot_take: rel rotate J1 back delta=%.2f\r\n", rel_back);
                }

                if (s.place_cmd_sent)
                {
                    s.place_stage = 5u;
                    s.place_cmd_sent = false;
                    s.settle_ms = now;
                }
            }
            else if (s.place_stage == 5u)
            {
                if ((now - s.settle_ms) >= ROBOT_TAKE_BASE_ROTATE_SETTLE_MS)
                {
                    s.place_stage = 6u;
                    s.settle_ms = 0u;
                }
            }
            else if (s.place_stage == 6u)
            {
                if (!s.place_cmd_sent)
                {
                    s.place_cmd_sent = (robot_send_reset_event(false) != 0);
                    s.settle_ms = now;
                    LOG("robot_take: soft reset cmd\r\n");
                }

                if (s.place_cmd_sent)
                {
                    s.place_stage = 7u;
                    s.place_cmd_sent = false;
                    s.settle_ms = now;
                }
            }
            else if (s.place_stage == 7u)
            {
                if (robot_is_soft_reset_done())
                {
                    s.place_stage = 8u;
                    s.settle_ms = 0u;
                    LOG("robot_take: soft reset done -> hard reset\r\n");
                }
                else if ((now - s.settle_ms) >= ROBOT_TAKE_SOFT_RESET_WAIT_MS)
                {
                    LOG("robot_take: wait soft reset done timeout, keep waiting...\r\n");
                    s.settle_ms = now;
                }
            }
            else if (s.place_stage == 8u)
            {
                if (!s.place_cmd_sent)
                {
                    s.place_cmd_sent = (robot_send_reset_event(true) != 0);
                    s.settle_ms = now;
                    LOG("robot_take: hard reset cmd\r\n");
                }

                if (s.place_cmd_sent)
                {
                    s.place_stage = 9u;
                    s.place_cmd_sent = false;
                    s.settle_ms = now;
                }
            }
            else if (s.place_stage == 9u)
            {
                if (robot_is_hard_reset_done())
                {
                    s.place_stage = 10u;
                    s.settle_ms = 0u;
                    s.k230_prestart_sent = 0u;
                    LOG("robot_take: hard reset done -> notify K230\r\n");
                }
                else if ((now - s.settle_ms) >= ROBOT_TAKE_HARD_RESET_WAIT_MS)
                {
                    LOG("robot_take: wait hard reset done timeout, keep waiting...\r\n");
                    s.settle_ms = now;
                }
            }
            else
            {
                if ((s.k230_prestart_sent < ROBOT_TAKE_K230_PRESTART_REPEAT) &&
                    ((s.settle_ms == 0u) || ((now - s.settle_ms) >= ROBOT_TAKE_K230_PRESTART_INTERVAL_MS)))
                {
                    bool sent_ok = k230_send_status_u8(RA6_FUNC_AT_PRESTART, 0x01u);
                    s.settle_ms = now;
                    if (sent_ok) {
                        s.k230_prestart_sent++;
                        LOG("K230_send_status[%u/%u]\r\n",
                            (unsigned)s.k230_prestart_sent, (unsigned)ROBOT_TAKE_K230_PRESTART_REPEAT);
                    } else {
                        LOG("K230_send_status failed, retry...\r\n");
                    }
                }

                if (s.k230_prestart_sent >= ROBOT_TAKE_K230_PRESTART_REPEAT)
                {
                    K230_LED_OFF; // 夹取流程结束，关闭补光灯
                    g_strawberry_picked_count++;
                    enter_state(TAKE_INIT, now);
                    LOG("robot_take: cycle done, picked=%lu\r\n", (unsigned long)g_strawberry_picked_count);
                }
            }
            break;
        }

        case TAKE_RECOVER:				//失败/超时恢复到安全位
        {
            /* stage0: stop+lift; stage1: back pre */
            if (s.recover_stage == 0u)
            {
                if (!s.target_active) {
                    (void) gripper_stop(6);
                    s.target = g_robot.cur_pos;
                    s.target.z = ROBOT_TAKE_Z_SAFE;
                    s.target_active = send_auto(&s.target);
                    LOG("robot_take: recover lift\r\n");
                }
                if (s.target_active && pos_near(&g_robot.cur_pos, &s.target, 2.0f)) {
                    s.target_active = false;
                    s.recover_stage = 1u;
                }
            }
            else
            {
                if (!s.target_active) {
                    s.target = s.pre;
                    s.target_active = send_auto(&s.target);
                    LOG("robot_take: recover back pre\r\n");
                }
                if (s.target_active && pos_near(&g_robot.cur_pos, &s.target, 2.0f)) {
                    (void) k230_send_status_u8(RA6_FUNC_AT_PRESTART, 0x01u);
                    K230_LED_OFF; // 异常恢复结束也关闭补光灯
                    enter_state(TAKE_WAIT_TARGET, now);
                    LOG("robot_take: recovered\r\n");
                }
            }
            break;
        }


        default:
            enter_state(TAKE_INIT, now);
            break;
    }
}
