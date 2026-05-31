#include "vision_service_thread.h"
#include "bsp_uart.h"
#include "bsp_led.h"
#include "K230_cmd.h"
#include "robot.h"
#include "robot-take.h"

/*
 * Vision event-driven skeleton.
 *
 * K230 -> RA6M5 provides:
 * - 0x02: strawberry integrity (1 byte)
 * - 0x03: coordinate delta (Cx,Cy) int16 (4 bytes)
 * - 0x04: harvestable flag (1 byte)
 */

typedef enum {
    VISION_STATE_IDLE = 0,
    VISION_STATE_TRACKING,
    VISION_STATE_READY_TO_HARVEST,
} vision_state_t;

typedef struct {
    uint8_t strawberry_complete; /* 0/1 */
    uint8_t harvestable;         /* 0/1 */
} vision_flags_t;

static void vision_handle_frame(const k230_frame_t *f, vision_state_t *state, vision_flags_t *flags)
{
    if (!f || !state || !flags) {
        return;
    }
 
    switch (f->func)
    {
        case K230_FUNC_STRAWBERRY_INTEGRITY:
            if (f->payload_len >= 1u) {
                // uint8_t old = flags->strawberry_complete;
                flags->strawberry_complete = (f->payload[0] == 0x01u) ? 1u : 0u;


//                /* - 画面有没有草莓需要连续观测。
//                 * - strawberry=0：每次收到都打印，便于你触发“无草莓抬升”逻辑验证。
//                 * - strawberry=1：仍按“变化才打印”，避免高频刷屏。
//                 */
//                if ((flags->strawberry_complete == 0u) || (old != flags->strawberry_complete)) {
//                    LOG("K230: strawberry=%u\n", (unsigned)flags->strawberry_complete);
//                }
            }
            break;


        case K230_FUNC_HARVESTABLE:
            if (f->payload_len >= 1u) {
                // uint8_t old = flags->harvestable;
                flags->harvestable = (f->payload[0] == 0x01u) ? 1u : 0u;


                /* - 0/1 都要能看到。
                 * - 当 K230 连续发 0 时，即使状态没变化，也要打印一次。
                 * - 当 K230 连续发 1 时，为了避免刷屏，仍按“变化才打印”。
                 */
//                if ((flags->harvestable == 0u) || (old != flags->harvestable)) {
//                    LOG("K230: harvestable=%u\n", (unsigned)flags->harvestable);
//                }

                if (flags->harvestable) {
                    *state = VISION_STATE_READY_TO_HARVEST;
                }
            }
            break;


        case K230_FUNC_COORDINATE_DATA:
            /* Coordinate data arrives continuously while target exists. */
            *state = VISION_STATE_TRACKING;
            break;

        default:
            break;
    }
}


/* vision_thread entry function */
void vision_service_thread_entry(void * pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    /* 启动保护：等待其他线程（尤其是 robot_thread）完成硬件/驱动锁的初始化。
     * 这能有效避免启动瞬间因 `LOG` 锁尚未创建而触发的断言卡死。
     */
    vTaskDelay(pdMS_TO_TICKS(100));

    k230_protocol_init();

    robot_take_init();

    /* 兼容现有的解析/打印逻辑：保留 state/flags，但主流程交给 APP/robot-take.c */
    vision_state_t state = VISION_STATE_IDLE;
    vision_flags_t flags = {0};

    k230_frame_t frame;
    k230_point_t pt = {0};
    uint8_t harvestable = 0u;



    /* 记录 0x02/0x03/0x04 最近一次更新时间，用于有效期判定 */
    uint32_t last_flag02_ms = 0;
    uint32_t last_coord_ms = 0;
    uint32_t last_flag04_ms = 0;

    // k230_strawberry_count_t count = {0};

    while (1)
    {
        uint32_t now_ms = HAL_GetTick();

        k230_protocol_process();

        /* 1) Handle generic frames */
        while (k230_get_latest_frame(&frame))
        {
            vision_handle_frame(&frame, &state, &flags);

            if (frame.func == K230_FUNC_STRAWBERRY_INTEGRITY) {
                last_flag02_ms = now_ms;
            }
            if (frame.func == K230_FUNC_HARVESTABLE) {
                last_flag04_ms = now_ms;
            }
        }

//        if (k230_get_latest_strawberry_count(&count))
//        {
//            LOG("K230: count ripe=%u unripe=%u\n", (unsigned)count.ripe, (unsigned)count.unripe);
//        }

        /* 2) Handle latest coordinates */
        if (k230_get_latest_coords(&pt))
        {
            last_coord_ms = now_ms;
        }

        if (k230_get_latest_harvestable(&harvestable))
        {
            flags.harvestable = (harvestable == 0x01u) ? 1u : 0u;
            last_flag04_ms = now_ms;
            if (flags.harvestable) {
                state = VISION_STATE_READY_TO_HARVEST;
            }
        }

        /* 3) Drive main harvest FSM */
        robot_take_obs_t obs = {0};
        obs.now_ms = now_ms;

        if ((last_coord_ms != 0u) && ((now_ms - last_coord_ms) <= ROBOT_TAKE_K230_COORD_VALID_MS)) {
            obs.has_coord = true;
            obs.dcx = pt.x;
            obs.dcy = pt.y;
        }

        if ((last_flag02_ms != 0u) && ((now_ms - last_flag02_ms) <= ROBOT_TAKE_K230_FLAG02_VALID_MS)) {
            obs.has_flag02 = true;
            obs.flag02 = flags.strawberry_complete;
        }

        if ((last_flag04_ms != 0u) && ((now_ms - last_flag04_ms) <= ROBOT_TAKE_K230_FLAG_VALID_MS)) {
            obs.has_flag04 = true;
            obs.flag04 = flags.harvestable;
        }

        robot_take_step(&obs);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
