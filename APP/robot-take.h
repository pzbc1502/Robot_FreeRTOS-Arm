#ifndef ROBOT_TAKE_H_
#define ROBOT_TAKE_H_

#include <stdint.h>
#include <stdbool.h>

#include "robot.h"  

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================ */
/*                     采摘主状态机 - 配置参数                   */
/* ============================================================ */

/* 预开始位 (基座坐标，单位mm) - 先给默认值，后续你再根据机构标定 */
#ifndef ROBOT_TAKE_PRE_X
#define ROBOT_TAKE_PRE_X           (0.0f)
#endif
#ifndef ROBOT_TAKE_PRE_Y
#define ROBOT_TAKE_PRE_Y           (-15.0f)
#endif
#ifndef ROBOT_TAKE_Z_SAFE
#define ROBOT_TAKE_Z_SAFE          (0.0f)
#endif

/* 草莓放置点 (基座坐标，单位mm) */
#ifndef ROBOT_TAKE_BOX_X
#define ROBOT_TAKE_BOX_X           (0.0f)
#endif
#ifndef ROBOT_TAKE_BOX_Y
#define ROBOT_TAKE_BOX_Y           (-1.0f)
#endif
#ifndef ROBOT_TAKE_BOX_Z
#define ROBOT_TAKE_BOX_Z           (0.0f)
#endif

/* 放置位底座(J1)旋转配置
 * - 采用“读取当前home角 + 旋转角度”得到放置目标绝对角度。
 * - ROBOT_TAKE_BOX_BASE_DIR：0=+方向，1=-方向。
 */
#ifndef ROBOT_TAKE_BOX_BASE_ROTATE_DEG
#define ROBOT_TAKE_BOX_BASE_ROTATE_DEG       (95.0f)
#endif
#ifndef ROBOT_TAKE_BOX_BASE_DIR
#define ROBOT_TAKE_BOX_BASE_DIR              (0u)
#endif

/* 放置阶段稳态/复位等待（ms） */
#ifndef ROBOT_TAKE_BASE_ROTATE_SETTLE_MS
#define ROBOT_TAKE_BASE_ROTATE_SETTLE_MS     (5000u)
#endif
#ifndef ROBOT_TAKE_SOFT_RESET_WAIT_MS
#define ROBOT_TAKE_SOFT_RESET_WAIT_MS        (3500u)
#endif
#ifndef ROBOT_TAKE_HARD_RESET_WAIT_MS
#define ROBOT_TAKE_HARD_RESET_WAIT_MS        (1500u)
#endif

/* 向K230回传“到预备位”重复发送参数 */
#ifndef ROBOT_TAKE_K230_PRESTART_REPEAT
#define ROBOT_TAKE_K230_PRESTART_REPEAT      (10u)
#endif
#ifndef ROBOT_TAKE_K230_PRESTART_INTERVAL_MS
#define ROBOT_TAKE_K230_PRESTART_INTERVAL_MS (50u)
#endif

/* Z扫查范围（你后续可改成标定值） */
#ifndef ROBOT_TAKE_Z_MIN
#define ROBOT_TAKE_Z_MIN           (0.0f)        /* 机械臂 Z 轴最低点（对准/搜索不得低于该值） */
#endif
#ifndef ROBOT_TAKE_Z_MAX
#define ROBOT_TAKE_Z_MAX           (5.0f)       /* 机械臂最高点 */
#endif
#ifndef ROBOT_TAKE_SCAN_DZ
#define ROBOT_TAKE_SCAN_DZ         (1.0f)        /* 机械臂扫描步进 */
#endif

/* 接近参数：
 * - ROBOT_TAKE_APPROACH_DY：每次逼近的 Y 轴步长(mm)，例如 -15 -> -16。
 * - ROBOT_TAKE_APPROACH_STEP_PERIOD_MS：两次逼近的最小时间间隔(ms)。
 * - ROBOT_TAKE_MAX_APPROACH_STEPS：单次采摘流程允许的最大逼近步数。
 */
#ifndef ROBOT_TAKE_APPROACH_DY
#define ROBOT_TAKE_APPROACH_DY     (3.0f)
#endif
#ifndef ROBOT_TAKE_APPROACH_STEP_PERIOD_MS
#define ROBOT_TAKE_APPROACH_STEP_PERIOD_MS   (300u)
#endif
#ifndef ROBOT_TAKE_MAX_APPROACH_STEPS
#define ROBOT_TAKE_MAX_APPROACH_STEPS        (50u)
#endif

/*** 重点 ***根据 dCx/dCy 做轻微对准微调（k230画面 x,y -> 机械臂三维位置 x,z） */
#ifndef ROBOT_TAKE_KX_MM_PER_PX
#define ROBOT_TAKE_KX_MM_PER_PX    (-0.38f)   /* 画面 x -> 机械臂 x */
#endif
#ifndef ROBOT_TAKE_KY_MM_PER_PX
#define ROBOT_TAKE_KY_MM_PER_PX    (0.35f)   /* 画面 y -> 机械臂 z */
#endif
#ifndef ROBOT_TAKE_XY_MAX_STEP_MM
#define ROBOT_TAKE_XY_MAX_STEP_MM  (1.5f)   /* 机械臂 x/z 最大调整量 */
#endif

/* 对准判定阈值（像素）  K230发送的对位结果，如果与期望偏差在范围内，则认为对位成功*/
#ifndef ROBOT_TAKE_ALIGN_TOL_PX
#define ROBOT_TAKE_ALIGN_TOL_PX    (30.0f)
#endif

/* 对准更新步进周期 */
#ifndef ROBOT_TAKE_ALIGN_STEP_PERIOD_MS
#define ROBOT_TAKE_ALIGN_STEP_PERIOD_MS     (500u)
#endif

/* 0x02/0x03 有效期（ms） */
#ifndef ROBOT_TAKE_K230_FLAG02_VALID_MS
#define ROBOT_TAKE_K230_FLAG02_VALID_MS     (500u)
#endif
#ifndef ROBOT_TAKE_K230_COORD_VALID_MS
#define ROBOT_TAKE_K230_COORD_VALID_MS      (500u)
#endif

/* 夹住后“向下摘取” */
#ifndef ROBOT_TAKE_PLUCK_DZ
#define ROBOT_TAKE_PLUCK_DZ        (0.0f)
#endif
#ifndef ROBOT_TAKE_PLUCK_HOLD_MS
#define ROBOT_TAKE_PLUCK_HOLD_MS   (500u)
#endif

/* K230 0x04 标志有效期：超过该时间没更新，认为丢失 */
#ifndef ROBOT_TAKE_K230_FLAG_VALID_MS
#define ROBOT_TAKE_K230_FLAG_VALID_MS        (500u)
#endif

/* 等待K230对准/判断的超时 */
#ifndef ROBOT_TAKE_WAIT_ALIGN_TIMEOUT_MS
#define ROBOT_TAKE_WAIT_ALIGN_TIMEOUT_MS     (5000u)
#endif

/* 非阻塞稳态等待参数（到位后再等待一小段时间，避免状态切换过快） */
//抬起
#ifndef ROBOT_TAKE_LIFT_SETTLE_MS
#define ROBOT_TAKE_LIFT_SETTLE_MS            (500u)
#endif
//放置：去箱子
#ifndef ROBOT_TAKE_PLACE_SETTLE_MS
#define ROBOT_TAKE_PLACE_SETTLE_MS           (500u)
#endif

#ifndef ROBOT_TAKE_PRE_SETTLE_MS
#define ROBOT_TAKE_PRE_SETTLE_MS             (500u)
#endif
#ifndef ROBOT_TAKE_GRIPPER_OPEN_HOLD_MS
#define ROBOT_TAKE_GRIPPER_OPEN_HOLD_MS      (500u)
#endif

/* ============================================================ */
/*                         观测输入结构                          */
/* ============================================================ */

typedef struct
{
    /* K230 坐标偏差（像素） */
    bool    has_coord;
    int16_t dcx;
    int16_t dcy;

    /* K230 0x02 功能位：
     * - 0：无草莓（触发 Z 上移/搜索）
     * - 1：有草莓（通常会直接跟随 0x03 坐标）
     */
    bool    has_flag02;
    uint8_t flag02;

    /* K230 0x04 功能位：
     * - 0：允许接近（K230认为已对准，但未到夹取位置）
     * - 1：立即夹取（K230已对准，到夹取位置）
     */
    bool    has_flag04;
    uint8_t flag04;

    /* 当前时间(ms) */
    uint32_t now_ms;
} robot_take_obs_t;

/* ============================================================ */
/*                           对外API                              */
/* ============================================================ */

extern volatile uint32_t g_strawberry_picked_count;

void robot_take_init(void);
void robot_take_step(const robot_take_obs_t *obs);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_TAKE_H_ */
