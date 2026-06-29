#include "robot_target.h"
#include "jetson_vision.h"
#include "bsp_laser.h"
#include "bsp_uart.h"
#include <math.h>
#include <string.h>

typedef enum
{
    TARGET_INIT = 0,
    TARGET_PRE_POSITION,
    TARGET_WAIT_DETECT,
    TARGET_ALIGN,
    TARGET_CONFIRM,
    TARGET_OUTPUT,
    TARGET_DONE,
    TARGET_RECOVER,
} target_state_t;

typedef struct
{
    target_state_t state;
    uint32_t enter_ms;
    uint32_t last_step_ms;
    uint32_t last_vision_ms;
    int16_t dcx;
    int16_t dcy;
    uint8_t stable_count;
    bool has_vision;
    struct position pre;
    struct position target;
} target_ctx_t;

volatile bool ROBOT_TARGET_ENABLED = false;
volatile bool ROBOT_TARGET_FIRE_ENABLE = false;

static target_ctx_t s_target;

static float clampf_local(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static bool robot_any_limit_triggered(void)
{
    for (uint32_t i = 0u; i < ROBOT_MAX_JOINT_NUM; i++)
    {
        if (ROBOT_STATUS_IS(g_robot.joints[i].status, ROBOT_STATUS_LIMIT_HAPPENED))
        {
            return true;
        }
    }
    return false;
}

static bool position_near(const struct position *a, const struct position *b)
{
    return (fabsf(a->x - b->x) <= 1.0f) &&
           (fabsf(a->y - b->y) <= 1.0f) &&
           (fabsf(a->z - b->z) <= 1.0f);
}

static bool vision_fresh(uint32_t now_ms)
{
    return s_target.has_vision && ((now_ms - s_target.last_vision_ms) <= TARGET_VISION_VALID_MS);
}

static bool align_in_tolerance(void)
{
    return (fabsf((float)s_target.dcx) <= TARGET_ALIGN_TOL_PX) &&
           (fabsf((float)s_target.dcy) <= TARGET_ALIGN_TOL_PX);
}

static void enter_state(target_state_t state, uint32_t now_ms)
{
    s_target.state = state;
    s_target.enter_ms = now_ms;
}

static void force_laser_off(void)
{
    BSP_Laser_Off();
}

static bool send_target_auto(const struct position *pos)
{
    struct position local = *pos;
    return (robot_send_auto_event(&local) == pdPASS);
}

void robot_target_init(void)
{
    memset(&s_target, 0, sizeof(s_target));
    s_target.pre.x = TARGET_PRE_X;
    s_target.pre.y = TARGET_PRE_Y;
    s_target.pre.z = TARGET_PRE_Z;
    s_target.target = s_target.pre;
    s_target.state = TARGET_INIT;
    force_laser_off();
}

bool robot_target_enable_request(void)
{
    force_laser_off();
    if (!ROBOT_STATUS_IS(g_robot.status, ROBOT_STATUS_POSE_VALID))
    {
        ROBOT_TARGET_ENABLED = false;
        return false;
    }

    ROBOT_TARGET_ENABLED = true;
    return true;
}

void robot_target_disable_request(void)
{
    ROBOT_TARGET_ENABLED = false;
    ROBOT_TARGET_FIRE_ENABLE = false;
    force_laser_off();
}

void robot_target_step(const target_obs_t *obs)
{
    uint32_t now = (obs != NULL) ? obs->now_ms : HAL_GetTick();
    bool estop = (obs != NULL) && obs->estop_active;
    bool limit = ((obs != NULL) && obs->limit_triggered) || robot_any_limit_triggered();
    bool fire_allowed = ((obs != NULL) && obs->fire_button) || ROBOT_TARGET_FIRE_ENABLE;
    bool new_vision = (obs != NULL) && obs->has_vision;

    if (new_vision)
    {
        s_target.dcx = obs->dcx;
        s_target.dcy = obs->dcy;
        s_target.last_vision_ms = now;
        s_target.has_vision = true;
    }

    if (!ROBOT_TARGET_ENABLED)
    {
        enter_state(TARGET_INIT, now);
        return;
    }

    if (estop || limit)
    {
        force_laser_off();
        (void)jetson_send_status_u8(RA6_TO_JETSON_ERROR, 1u);
        enter_state(TARGET_RECOVER, now);
        return;
    }

    switch (s_target.state)
    {
        case TARGET_INIT:
            force_laser_off();
            s_target.target = s_target.pre;
            s_target.stable_count = 0u;
            if (send_target_auto(&s_target.target))
            {
                enter_state(TARGET_PRE_POSITION, now);
            }
            break;

        case TARGET_PRE_POSITION:
            force_laser_off();
            if (position_near(&g_robot.cur_pos, &s_target.target))
            {
                (void)jetson_send_status_u8(RA6_TO_JETSON_READY, 1u);
                enter_state(TARGET_WAIT_DETECT, now);
            }
            break;

        case TARGET_WAIT_DETECT:
            force_laser_off();
            if (vision_fresh(now))
            {
                enter_state(TARGET_ALIGN, now);
            }
            break;

        case TARGET_ALIGN:
            force_laser_off();
            if (!vision_fresh(now))
            {
                s_target.stable_count = 0u;
                enter_state(TARGET_RECOVER, now);
                break;
            }

            if (align_in_tolerance())
            {
                if (new_vision && (s_target.stable_count < TARGET_ALIGN_STABLE_COUNT))
                {
                    s_target.stable_count++;
                }
                if (s_target.stable_count >= TARGET_ALIGN_STABLE_COUNT)
                {
                    (void)jetson_send_status_u8(RA6_TO_JETSON_ALIGN_DONE, 1u);
                    enter_state(TARGET_CONFIRM, now);
                }
                break;
            }

            s_target.stable_count = 0u;
            if ((now - s_target.last_step_ms) >= TARGET_ALIGN_PERIOD_MS)
            {
                float dx = clampf_local((float)s_target.dcx * TARGET_KX_MM_PER_PX,
                                        -TARGET_MAX_STEP_MM, TARGET_MAX_STEP_MM);
                float dz = clampf_local((float)s_target.dcy * TARGET_KY_MM_PER_PX,
                                        -TARGET_MAX_STEP_MM, TARGET_MAX_STEP_MM);

                s_target.target = g_robot.cur_pos;
                s_target.target.x += dx;
                s_target.target.z += dz;
                if (send_target_auto(&s_target.target))
                {
                    s_target.last_step_ms = now;
                }
            }
            break;

        case TARGET_CONFIRM:
            force_laser_off();
            if (!vision_fresh(now))
            {
                enter_state(TARGET_RECOVER, now);
                break;
            }
            if (fire_allowed)
            {
                (void)jetson_send_status_u8(RA6_TO_JETSON_OUTPUT, 1u);
                enter_state(TARGET_OUTPUT, now);
            }
            break;

        case TARGET_OUTPUT:
            if (!fire_allowed || !vision_fresh(now))
            {
                force_laser_off();
                (void)jetson_send_status_u8(RA6_TO_JETSON_OUTPUT, 0u);
                enter_state(TARGET_DONE, now);
                break;
            }
            taskENTER_CRITICAL();
            bool enabled_now = ROBOT_TARGET_ENABLED;
            if (enabled_now)
            {
                BSP_Laser_On();
            }
            taskEXIT_CRITICAL();
            if (!enabled_now)
            {
                force_laser_off();
                enter_state(TARGET_INIT, now);
            }
            break;

        case TARGET_DONE:
            force_laser_off();
            if (!fire_allowed)
            {
                s_target.stable_count = 0u;
                enter_state(TARGET_WAIT_DETECT, now);
            }
            break;

        case TARGET_RECOVER:
        default:
            force_laser_off();
            if ((now - s_target.enter_ms) >= TARGET_ALIGN_PERIOD_MS)
            {
                s_target.stable_count = 0u;
                enter_state(TARGET_WAIT_DETECT, now);
            }
            break;
    }
}
