#ifndef ROBOT_TARGET_H_
#define ROBOT_TARGET_H_

#include <stdint.h>
#include <stdbool.h>
#include "robot.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TARGET_PRE_X
#define TARGET_PRE_X                 (0.0f)
#endif
#ifndef TARGET_PRE_Y
#define TARGET_PRE_Y                 (-15.0f)
#endif
#ifndef TARGET_PRE_Z
#define TARGET_PRE_Z                 (0.0f)
#endif

#define TARGET_KX_MM_PER_PX          (-0.38f)
#define TARGET_KY_MM_PER_PX          (0.35f)
#define TARGET_MAX_STEP_MM           (1.5f)
#define TARGET_ALIGN_TOL_PX          (30.0f)
#define TARGET_ALIGN_STABLE_COUNT    (3u)
#define TARGET_ALIGN_PERIOD_MS       (200u)
#define TARGET_VISION_VALID_MS       (500u)

typedef struct
{
    bool has_vision;
    int16_t dcx;
    int16_t dcy;
    uint32_t now_ms;
    bool estop_active;
    bool limit_triggered;
    bool fire_button;
} target_obs_t;

extern volatile bool ROBOT_TARGET_ENABLED;
extern volatile bool ROBOT_TARGET_FIRE_ENABLE;

void robot_target_init(void);
void robot_target_step(const target_obs_t *obs);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_TARGET_H_ */
