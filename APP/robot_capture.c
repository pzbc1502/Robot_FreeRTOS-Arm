#include "robot_capture.h"
#include "robot.h"
#include "robot_target.h"
#include "jetson_vision.h"
#include "bsp_laser.h"
#include "bsp_uart.h"
#include "hal_data.h"
#include "FreeRTOS.h"
#include <string.h>

#define CAPTURE_STEP_TIMEOUT_MS 10000u
#define CAPTURE_JOINT_TIMEOUT_MS 5000u
#define CAPTURE_RESET_TIMEOUT_MS 12000u

typedef enum
{
    CAPTURE_IDLE = 0,
    CAPTURE_START_STEP,
    CAPTURE_WAIT_AUTO,
    CAPTURE_WAIT_JOINT,
    CAPTURE_WAIT_RESET,
    CAPTURE_SAFE_RECOVER,
} capture_state_t;

typedef enum
{
    CAPTURE_STEP_AUTO = 0,
    CAPTURE_STEP_ABS,
    CAPTURE_STEP_RESET,
    CAPTURE_STEP_DONE,
} capture_step_type_t;

typedef struct
{
    capture_step_type_t type;
    uint8_t joint_id;
    float value;
} capture_step_t;

typedef struct
{
    capture_state_t state;
    uint8_t action;
    uint8_t point_id;
    uint8_t step_index;
    uint32_t step_start_ms;
    uint32_t last_distance_ms;
    uint16_t distance_mm;
    bool has_distance;
    bool distance_valid;
    bool distance_too_close;
    bool sent_safe_error;
    bool base_pose_ready;
} capture_ctx_t;

static const capture_step_t s_left_steps[] =
{
    { CAPTURE_STEP_AUTO, 0u, 0.0f },
    { CAPTURE_STEP_ABS,  0u, 65.0f },
    { CAPTURE_STEP_ABS,  3u, 330.0f },
    { CAPTURE_STEP_ABS,  4u, 85.0f },
    { CAPTURE_STEP_DONE, 0u, 0.0f },
};

static const capture_step_t s_front_steps[] =
{
    { CAPTURE_STEP_AUTO, 0u, 0.0f },
    { CAPTURE_STEP_ABS,  0u, 90.0f },
    { CAPTURE_STEP_ABS,  3u, 0.0f },
    { CAPTURE_STEP_ABS,  4u, 80.0f },
    { CAPTURE_STEP_DONE, 0u, 0.0f },
};

static const capture_step_t s_right_steps[] =
{
    { CAPTURE_STEP_AUTO, 0u, 0.0f },
    { CAPTURE_STEP_ABS,  0u, 115.0f },
    { CAPTURE_STEP_ABS,  3u, 35.0f },
    { CAPTURE_STEP_ABS,  4u, 80.0f },
    { CAPTURE_STEP_DONE, 0u, 0.0f },
};

static const capture_step_t s_finish_steps[] =
{
    { CAPTURE_STEP_RESET, 0u, 0.0f },
    { CAPTURE_STEP_DONE,  0u, 0.0f },
};

static capture_ctx_t s_capture;

static void capture_laser_off(void)
{
    BSP_Laser_Off();
}

static bool capture_any_limit_triggered(void)
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

static bool capture_action_valid(uint8_t action, uint8_t point_id)
{
    if (action == JETSON_CAPTURE_ACTION_GOTO)
    {
        return (point_id >= 1u) && (point_id <= 3u);
    }

    if (action == JETSON_CAPTURE_ACTION_FINISH)
    {
        return point_id == 0u;
    }

    if (action == JETSON_CAPTURE_ACTION_SELECT)
    {
        return (point_id >= 1u) && (point_id <= 3u);
    }

    if (action == JETSON_CAPTURE_ACTION_CURRENT)
    {
        return point_id == 0u;
    }

    return false;
}

static const capture_step_t *capture_steps_for(uint8_t action, uint8_t point_id)
{
    if (action == JETSON_CAPTURE_ACTION_FINISH)
    {
        return s_finish_steps;
    }

    switch (point_id)
    {
        case 1u: return s_left_steps;
        case 2u: return s_front_steps;
        case 3u: return s_right_steps;
        default: return NULL;
    }
}

static bool capture_distance_fresh(uint32_t now_ms)
{
    return s_capture.has_distance &&
           s_capture.distance_valid &&
           ((now_ms - s_capture.last_distance_ms) <= TARGET_SAFE_DISTANCE_VALID_MS);
}

static bool capture_distance_safe(uint32_t now_ms)
{
    return capture_distance_fresh(now_ms) && !s_capture.distance_too_close;
}

static void capture_update_distance(uint32_t now_ms, uint16_t distance_mm, bool valid)
{
    bool was_too_close = s_capture.distance_too_close;

    s_capture.has_distance = true;
    s_capture.distance_valid = valid;
    s_capture.distance_mm = distance_mm;
    s_capture.last_distance_ms = now_ms;

    if (valid)
    {
        if (distance_mm < TARGET_SAFE_DISTANCE_MM)
        {
            s_capture.distance_too_close = true;
        }
        else if (distance_mm >= TARGET_SAFE_DISTANCE_RELEASE_MM)
        {
            s_capture.distance_too_close = false;
        }
    }
    else
    {
        s_capture.distance_too_close = false;
    }

    if (was_too_close != s_capture.distance_too_close)
    {
        (void)jetson_send_status_u8(RA6_TO_JETSON_SAFE_DISTANCE,
                                    s_capture.distance_too_close ? 0u : 1u);
    }
}

static void capture_abort(uint8_t error_code)
{
    capture_laser_off();
    robot_motion_abort();
    robot_visual_servo_stop();
    (void)jetson_send_status_u8(RA6_TO_JETSON_ERROR, error_code);
    s_capture.state = CAPTURE_IDLE;
    s_capture.action = 0u;
    s_capture.point_id = 0u;
    s_capture.step_index = 0u;
    s_capture.base_pose_ready = false;
}

static bool capture_handle_safety(uint32_t now_ms)
{
    if (s_capture.action == JETSON_CAPTURE_ACTION_FINISH)
    {
        return false;
    }

    if (!capture_distance_fresh(now_ms))
    {
        capture_abort(JETSON_ERROR_SAFETY);
        return true;
    }

    if (!s_capture.distance_too_close)
    {
        return false;
    }

    capture_laser_off();
    robot_motion_abort();
    s_capture.base_pose_ready = false;
    if (!s_capture.sent_safe_error)
    {
        (void)jetson_send_status_u8(RA6_TO_JETSON_SAFE_DISTANCE, 0u);
        (void)jetson_send_status_u8(RA6_TO_JETSON_ERROR, JETSON_ERROR_SAFE_DISTANCE_TOO_CLOSE);
        s_capture.sent_safe_error = true;
    }

    if (!robot_is_visual_servo_active())
    {
        (void)robot_visual_servo_start();
    }
    robot_visual_servo_set_velocity(0.0f, TARGET_SAFE_RETREAT_SPEED_MM_S, 0.0f);
    s_capture.state = CAPTURE_SAFE_RECOVER;
    return true;
}

static void capture_finish(uint32_t now_ms)
{
    (void)now_ms;
    capture_laser_off();
    robot_visual_servo_stop();

    if (s_capture.action == JETSON_CAPTURE_ACTION_GOTO)
    {
        (void)jetson_send_status_u8(RA6_TO_JETSON_CAPTURE_POINT, s_capture.point_id);
    }
    else if (s_capture.action == JETSON_CAPTURE_ACTION_FINISH)
    {
        (void)jetson_send_status_u8(RA6_TO_JETSON_CAPTURE_DONE, 1u);
        s_capture.base_pose_ready = false;
    }
    else if (s_capture.action == JETSON_CAPTURE_ACTION_SELECT)
    {
        robot_target_mark_preposition_ready_once();
        (void)jetson_send_status_u8(RA6_TO_JETSON_TARGET_PRESTART, s_capture.point_id);
    }

    s_capture.state = CAPTURE_IDLE;
    s_capture.action = 0u;
    s_capture.point_id = 0u;
    s_capture.step_index = 0u;
}

static void capture_start_current_step(uint32_t now_ms)
{
    const capture_step_t *steps = capture_steps_for(s_capture.action, s_capture.point_id);
    if (steps == NULL)
    {
        capture_abort(JETSON_ERROR_INVALID_PARAM);
        return;
    }

    const capture_step_t *step = &steps[s_capture.step_index];
    s_capture.step_start_ms = now_ms;

    switch (step->type)
    {
        case CAPTURE_STEP_AUTO:
        {
            if (s_capture.base_pose_ready)
            {
                LOG("[CAPTURE] base pose ready, skip repeated auto\r\n");
                s_capture.step_index++;
                s_capture.state = CAPTURE_START_STEP;
                break;
            }

            struct position pos = { .x = 0.0f, .y = -130.0f, .z = -15.0f };
            if (robot_send_auto_event(&pos) == pdPASS)
            {
                s_capture.state = CAPTURE_WAIT_AUTO;
            }
            else
            {
                capture_abort(JETSON_ERROR_SAFETY);
            }
            break;
        }

        case CAPTURE_STEP_ABS:
            if (robot_send_abs_rotate_event(step->joint_id, step->value) == pdPASS)
            {
                s_capture.state = CAPTURE_WAIT_JOINT;
            }
            else
            {
                capture_abort(JETSON_ERROR_SAFETY);
            }
            break;

        case CAPTURE_STEP_RESET:
            if (robot_send_reset_event(false) == pdPASS)
            {
                s_capture.state = CAPTURE_WAIT_RESET;
            }
            else
            {
                capture_abort(JETSON_ERROR_SAFETY);
            }
            break;

        case CAPTURE_STEP_DONE:
            capture_finish(now_ms);
            break;

        default:
            capture_abort(JETSON_ERROR_INVALID_PARAM);
            break;
    }
}

void robot_capture_init(void)
{
    memset(&s_capture, 0, sizeof(s_capture));
    s_capture.state = CAPTURE_IDLE;
    capture_laser_off();
}

bool robot_capture_request(uint8_t action, uint8_t point_id)
{
    if (!capture_action_valid(action, point_id))
    {
        (void)jetson_send_status_u8(RA6_TO_JETSON_ERROR, JETSON_ERROR_INVALID_PARAM);
        return false;
    }

    if ((s_capture.state != CAPTURE_IDLE) ||
        ROBOT_TARGET_ENABLED ||
        robot_is_auto_busy() ||
        robot_is_visual_servo_active())
    {
        (void)jetson_send_status_u8(RA6_TO_JETSON_ERROR, JETSON_ERROR_BUSY);
        return false;
    }

    if (action == JETSON_CAPTURE_ACTION_CURRENT)
    {
        capture_laser_off();
        robot_target_mark_preposition_ready_once();
        (void)jetson_send_status_u8(RA6_TO_JETSON_TARGET_PRESTART, 0u);
        LOG("[CAPTURE] current pose marked as target prestart\r\n");
        return true;
    }

    s_capture.action = action;
    s_capture.point_id = point_id;
    s_capture.step_index = 0u;
    s_capture.step_start_ms = 0u;
    s_capture.sent_safe_error = false;
    s_capture.state = CAPTURE_START_STEP;
    capture_laser_off();

    LOG("[CAPTURE] request action=%u point=%u\r\n",
        (unsigned)action,
        (unsigned)point_id);
    return true;
}

void robot_capture_step(const robot_capture_obs_t *obs)
{
    uint32_t now = (obs != NULL) ? obs->now_ms : HAL_GetTick();

    if (obs != NULL && obs->has_distance)
    {
        capture_update_distance(now, obs->distance_mm, obs->distance_valid);
    }

    if (s_capture.state == CAPTURE_IDLE)
    {
        return;
    }

    if ((obs != NULL) && obs->estop_active)
    {
        capture_abort(JETSON_ERROR_HEARTBEAT_TIMEOUT);
        return;
    }

    if (((obs != NULL) && obs->limit_triggered) || capture_any_limit_triggered())
    {
        capture_abort(JETSON_ERROR_SAFETY);
        return;
    }

    if (s_capture.state == CAPTURE_SAFE_RECOVER)
    {
        if (capture_distance_safe(now))
        {
            robot_visual_servo_stop();
            s_capture.state = CAPTURE_IDLE;
            s_capture.action = 0u;
            s_capture.point_id = 0u;
            s_capture.step_index = 0u;
            (void)jetson_send_status_u8(RA6_TO_JETSON_SAFE_DISTANCE, 1u);
        }
        else if (!capture_distance_fresh(now))
        {
            robot_visual_servo_stop();
            s_capture.state = CAPTURE_IDLE;
        }
        else
        {
            if (!robot_is_visual_servo_active())
            {
                (void)robot_visual_servo_start();
            }
            if (robot_is_visual_servo_active())
            {
                robot_visual_servo_set_velocity(0.0f, TARGET_SAFE_RETREAT_SPEED_MM_S, 0.0f);
            }
        }
        return;
    }

    if (capture_handle_safety(now))
    {
        return;
    }

    switch (s_capture.state)
    {
        case CAPTURE_START_STEP:
            capture_start_current_step(now);
            break;

        case CAPTURE_WAIT_AUTO:
        {
            robot_auto_result_t result = robot_auto_result_consume();
            if (result == ROBOT_AUTO_RESULT_OK)
            {
                s_capture.base_pose_ready = true;
                s_capture.step_index++;
                s_capture.state = CAPTURE_START_STEP;
            }
            else if ((result == ROBOT_AUTO_RESULT_FAILED) ||
                     (result == ROBOT_AUTO_RESULT_ABORTED) ||
                     ((now - s_capture.step_start_ms) > CAPTURE_STEP_TIMEOUT_MS))
            {
                capture_abort(JETSON_ERROR_SAFETY);
            }
            break;
        }

        case CAPTURE_WAIT_JOINT:
        {
            robot_joint_result_t result = robot_joint_result_consume();
            if (result == ROBOT_JOINT_RESULT_OK)
            {
                s_capture.step_index++;
                s_capture.state = CAPTURE_START_STEP;
            }
            else if ((result == ROBOT_JOINT_RESULT_FAILED) ||
                     (result == ROBOT_JOINT_RESULT_ABORTED) ||
                     ((now - s_capture.step_start_ms) > CAPTURE_JOINT_TIMEOUT_MS))
            {
                capture_abort(JETSON_ERROR_SAFETY);
            }
            break;
        }

        case CAPTURE_WAIT_RESET:
        {
            robot_reset_result_t result = robot_reset_result_consume();
            if (result == ROBOT_RESET_RESULT_OK)
            {
                s_capture.step_index++;
                s_capture.state = CAPTURE_START_STEP;
            }
            else if ((result == ROBOT_RESET_RESULT_FAILED) ||
                     ((now - s_capture.step_start_ms) > CAPTURE_RESET_TIMEOUT_MS))
            {
                capture_abort(JETSON_ERROR_SAFETY);
            }
            break;
        }

        default:
            break;
    }
}

bool robot_capture_is_active(void)
{
    return s_capture.state != CAPTURE_IDLE;
}
