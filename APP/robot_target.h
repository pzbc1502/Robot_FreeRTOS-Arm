#ifndef ROBOT_TARGET_H_
#define ROBOT_TARGET_H_

#include <stdint.h>
#include <stdbool.h>
#include "robot.h"

#ifdef __cplusplus
extern "C" {
#endif

//预开始位置
#ifndef TARGET_PRE_X
#define TARGET_PRE_X                 (0.0f)      /* 定靶预定位 X 方向偏移，单位 mm */
#endif
#ifndef TARGET_PRE_Y
#define TARGET_PRE_Y                 (-60.0f)    /* 定靶预定位 Y 方向偏移，单位 mm，负值表示向前靠近目标 */
#endif
#ifndef TARGET_PRE_Z
#define TARGET_PRE_Z                 (-1.5f)      /* 定靶预定位 Z 方向偏移，单位 mm */
#endif



//视觉伺服开关
#ifndef TARGET_USE_VISUAL_SERVO
#define TARGET_USE_VISUAL_SERVO      (1)        /* 1=使用视觉伺服闭环，0=不使用视觉伺服闭环 */
#endif

//视觉伺服参数
#define TARGET_VS_KX_MM_S_PER_PX     (0.35f)
#define TARGET_VS_KZ_MM_S_PER_PX     (0.35f)
#define TARGET_VS_MAX_SPEED_MM_S     (8.0f)
#define TARGET_VS_FINE_MAX_SPEED_MM_S (3.0f)
#define TARGET_VS_CMD_TIMEOUT_MS     (250u)
#define TARGET_SAFE_DISTANCE_MM          (100u)
#define TARGET_SAFE_DISTANCE_RELEASE_MM  (110u)
#define TARGET_SAFE_DISTANCE_VALID_MS    (500u)
#define TARGET_SAFE_RETREAT_SPEED_MM_S   (6.0f)



//末端auto修正参数
#define TARGET_KX_MM_PER_PX          (0.38f)    /* 视觉 dcx 到机械臂 X 小步修正的比例，单位 mm/px */
#define TARGET_KY_MM_PER_PX          (0.35f)     /* 视觉 dcy 到机械臂 Z 小步修正的比例，单位 mm/px */
#define TARGET_MAX_STEP_MM           (2.5f)      /* 单次视觉对准最大修正步长，单位 mm */
#define TARGET_ALIGN_TOL_PX_COARSE   (15.0f)     /* 粗/精定位切换阈值，单位 px */
#define TARGET_MAX_STEP_MM_FINE      (0.8f)      /* 精定位阶段单次最大修正步长，单位 mm */
#define TARGET_ALIGN_TOL_PX          (5.0f)     /* 视觉误差进入该像素阈值内视为单帧对准 */
#define TARGET_ALIGN_STABLE_COUNT    (3u)        /* 连续满足对准阈值的帧数，达到后进入确认状态 */
#define TARGET_CONFIRM_STABLE_COUNT  (4u)        /* 输出前二次确认所需连续新视觉帧数 */
#define TARGET_ALIGN_PERIOD_MS       (250u)      /* 视觉闭环小步修正周期，单位 ms */
#define TARGET_VISION_VALID_MS       (500u)      /* 视觉数据有效期，超时后退出对准/输出 */


typedef struct
{
    bool has_vision;
    int16_t dcx;
    int16_t dcy;
    bool has_distance;
    bool distance_valid;
    uint16_t distance_mm;
    uint32_t now_ms;
    bool estop_active;
    bool limit_triggered;
    bool fire_button;
} target_obs_t;

extern volatile bool ROBOT_TARGET_ENABLED;
extern volatile bool ROBOT_TARGET_FIRE_ENABLE;

void robot_target_init(void);
bool robot_target_enable_request(void);
void robot_target_disable_request(void);
void robot_target_step(const target_obs_t *obs);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_TARGET_H_ */
