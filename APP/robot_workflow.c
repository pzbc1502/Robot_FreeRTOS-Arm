#include "robot_workflow.h"
#include "robot.h"
#include "robot_capture.h"
#include "robot_target.h"
#include "jetson_vision.h"
#include "bsp_laser.h"
#include "bsp_led.h"
#include "bsp_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

#define WORKFLOW_COMMAND_CACHE_SIZE   (8u)
#define WORKFLOW_COMMAND_CACHE_TTL_MS (3000u)

typedef enum
{
    RETREAT_PHASE_IDLE = 0,
    RETREAT_PHASE_MOVING,
    RETREAT_PHASE_WAIT_DISTANCE,
    RETREAT_PHASE_DONE,
} retreat_phase_t;

typedef struct
{
    bool used;
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[2];
    uint32_t age;
    uint32_t received_ms;
    bool ack_valid;
    uint8_t ack_value;
    uint8_t ack_error;
    bool business_valid;
    uint8_t business_event;
    uint8_t business_value;
    uint8_t business_error;
} command_cache_entry_t;

typedef struct
{
    workflow_state_t state;
    uint8_t selected_view_id;
    uint8_t capture_next_point;
    uint8_t capture_done_mask;
    uint8_t safe_sample_count;
    uint8_t safe_sample_seqs[ROBOT_WORKFLOW_SAFE_STABLE_COUNT];
    uint8_t retreat_steps;
    uint8_t workflow_seq;
    uint8_t target_seq;
    uint8_t capture_seq;
    uint8_t capture_action;
    uint8_t capture_point_id;
    uint32_t state_deadline_ms;
    bool distance_safe_latched;
    bool target_enabled;
    bool target_aligned;
    bool fault_latched;
    bool laser_active;
    retreat_phase_t retreat_phase;
} robot_workflow_ctx_t;

typedef enum
{
    COMMAND_NEW = 0,
    COMMAND_DUPLICATE,
    COMMAND_CONFLICT,
    COMMAND_BUSY,
} command_prepare_result_t;

static robot_workflow_ctx_t s_workflow;
static command_cache_entry_t s_command_cache[WORKFLOW_COMMAND_CACHE_SIZE];
static uint32_t s_command_cache_age = 0u;
static struct
{
    bool pending, restart_locked;
    uint8_t stable_count;
    uint32_t deadline, next_sample, anchor_ms;
    float anchor[ROBOT_ARM_JOINT_NUM];
} s_stop;

/* Round identity survives command-cache expiry. IDs increase within one boot. */
static struct
{
    uint32_t id, high_water, deadline, next_verify;
    uint8_t state, flags, error, start_seq, photo_seq, photo_value;
    bool photo_seen;
} s_scan;

static void scan_report(uint8_t seq, uint8_t event, uint8_t action,
                        uint8_t result, uint8_t error)
{
    (void)jetson_send_scan_status(seq, event, s_scan.state, s_scan.flags,
                                  action, result, error, s_scan.id);
}

static void scan_invalidate(uint8_t state, uint8_t error)
{
    if ((s_scan.id != 0u) && (s_scan.state != JETSON_SCAN_NONE) &&
        ((s_scan.state != JETSON_SCAN_ABORTED_HOLD) ||
         (state == JETSON_SCAN_FAULT_HOLD)) &&
        (s_scan.state != JETSON_SCAN_FAULT_HOLD) &&
        (s_scan.state != JETSON_SCAN_CHAIN_COMPLETED))
    {
        s_scan.state = state;
        s_scan.flags = 0u;
        s_scan.error = error;
        scan_report(s_scan.start_seq, JETSON_SCAN_TERMINATED,
                    JETSON_SCAN_START, 0u, error);
    }
}

static void scan_snapshot(uint8_t seq, uint8_t action)
{
    uint8_t result = ((s_scan.state == JETSON_SCAN_ABORTED_HOLD) ||
                      (s_scan.state == JETSON_SCAN_FAULT_HOLD)) ? 0u : 1u;
    scan_report(seq, JETSON_SCAN_SNAPSHOT, action, result, s_scan.error);
}

static const char *workflow_state_name(workflow_state_t state)
{
    switch (state)
    {
        case FLOW_IDLE:                       return "IDLE";
        case FLOW_HOMING:                     return "HOMING";
        case FLOW_MEASURE_POSITION:            return "MEASURE_POSITION";
        case FLOW_WAIT_SAFE_DISTANCE:          return "WAIT_SAFE_DISTANCE";
        case FLOW_SAFE_READY:                  return "SAFE_READY";
        case FLOW_CAPTURE:                     return "CAPTURE";
        case FLOW_CAPTURE_HOME:                return "CAPTURE_HOME";
        case FLOW_SELECTED_VIEW:               return "SELECTED_VIEW";
        case FLOW_TARGET_ACTIVE:               return "TARGET_ACTIVE";
        case FLOW_TARGET_HOLD:                 return "TARGET_HOLD";
        case FLOW_RETURN_HOME:                 return "RETURN_HOME";
        case FLOW_RETREAT_WAIT_RESTART:        return "RETREAT_WAIT_RESTART";
        case FLOW_ABORT_HOLD:                  return "ABORT_HOLD";
        case FLOW_FAULT_HOLD:                  return "FAULT_HOLD";
        case FLOW_SCAN_ACTIVE:                 return "SCAN_ACTIVE";
        case FLOW_SCAN_READY:                  return "SCAN_READY";
        default:                               return "UNKNOWN";
    }
}

static void workflow_update_target_indicators(void)
{
    bool ready = (s_workflow.state == FLOW_TARGET_ACTIVE) &&
                 s_workflow.target_enabled && s_workflow.target_aligned;
    bool output = ready && s_workflow.laser_active;

    BSP_TargetReadyLed_Set(ready);
    BSP_TargetOutputLed_Set(output);
}

static void workflow_clear_safe_samples(void)
{
    s_workflow.safe_sample_count = 0u;
    memset(s_workflow.safe_sample_seqs, 0, sizeof(s_workflow.safe_sample_seqs));
}

static bool workflow_safe_seq_seen(uint8_t seq)
{
    for (uint8_t i = 0u; i < s_workflow.safe_sample_count; i++)
    {
        if (s_workflow.safe_sample_seqs[i] == seq)
        {
            return true;
        }
    }
    return false;
}

static void workflow_enter(workflow_state_t state, uint32_t now_ms)
{
    if (s_workflow.state != state)
    {
        LOG("[WORKFLOW] %s -> %s\r\n",
            workflow_state_name(s_workflow.state), workflow_state_name(state));
    }
    s_workflow.state = state;
    s_workflow.state_deadline_ms = now_ms;
    workflow_update_target_indicators();
}

static bool workflow_any_limit_triggered(void)
{
    for (uint8_t i = 0u; i < ROBOT_MAX_JOINT_NUM; i++)
    {
        if (ROBOT_STATUS_IS(g_robot.joints[i].status, ROBOT_STATUS_LIMIT_HAPPENED))
        {
            return true;
        }
    }
    return false;
}

static bool workflow_is_active(void)
{
    return (s_workflow.state != FLOW_IDLE) &&
           (s_workflow.state != FLOW_ABORT_HOLD) &&
           (s_workflow.state != FLOW_FAULT_HOLD);
}

static bool command_payload_equal(const command_cache_entry_t *entry,
                                  const uint8_t *payload, uint8_t len)
{
    if ((entry == NULL) || (entry->len != len))
    {
        return false;
    }
    return (len == 0u) || (memcmp(entry->payload, payload, len) == 0);
}

static command_cache_entry_t *command_find(uint8_t type, uint8_t seq)
{
    for (uint8_t i = 0u; i < WORKFLOW_COMMAND_CACHE_SIZE; i++)
    {
        if (s_command_cache[i].used &&
            (s_command_cache[i].type == type) &&
            (s_command_cache[i].seq == seq))
        {
            return &s_command_cache[i];
        }
    }
    return NULL;
}

static bool command_stop_pending(const command_cache_entry_t *entry)
{
    return s_stop.pending && entry->used &&
           (entry->type == JETSON_MSG_WORKFLOW_CTRL) && (entry->len == 1u) &&
           (entry->payload[0] == JETSON_WORKFLOW_ACTION_ABORT_HOLD) &&
           entry->ack_valid && (entry->ack_value == 1u) && !entry->business_valid;
}

static void command_replay(const command_cache_entry_t *entry)
{
    if (entry == NULL)
    {
        return;
    }
    if (entry->ack_valid)
    {
        (void)jetson_send_status(entry->seq, RA6_TO_JETSON_COMMAND_ACK,
                                 entry->ack_value, entry->ack_error);
    }
    if (entry->business_valid)
    {
        (void)jetson_send_status(entry->seq, entry->business_event,
                                 entry->business_value, entry->business_error);
    }
}

static command_cache_entry_t *command_prepare(uint8_t type, uint8_t seq,
                                               const uint8_t *payload, uint8_t len,
                                               command_prepare_result_t *result,
                                               uint32_t now_ms)
{
    command_cache_entry_t *entry = command_find(type, seq);
    if ((entry != NULL) &&
        !command_stop_pending(entry) &&
        ((uint32_t)(now_ms - entry->received_ms) > WORKFLOW_COMMAND_CACHE_TTL_MS))
    {
        memset(entry, 0, sizeof(*entry));
        entry = NULL;
    }
    if (entry != NULL)
    {
        if (command_payload_equal(entry, payload, len))
        {
            command_replay(entry);
            *result = COMMAND_DUPLICATE;
        }
        else
        {
            (void)jetson_send_status(seq, RA6_TO_JETSON_COMMAND_ACK, 0u,
                                     JETSON_ERROR_SEQ_CONFLICT);
            *result = COMMAND_CONFLICT;
        }
        return entry;
    }

    uint8_t slot = WORKFLOW_COMMAND_CACHE_SIZE;
    uint32_t oldest_age = UINT32_MAX;
    for (uint8_t i = 0u; i < WORKFLOW_COMMAND_CACHE_SIZE; i++)
    {
        if (!s_command_cache[i].used)
        {
            slot = i;
            oldest_age = 0u;
            break;
        }
        if (!command_stop_pending(&s_command_cache[i]) &&
            (s_command_cache[i].age < oldest_age))
        {
            oldest_age = s_command_cache[i].age;
            slot = i;
        }
    }

    if (slot == WORKFLOW_COMMAND_CACHE_SIZE)
    {
        (void)jetson_send_status(seq, RA6_TO_JETSON_COMMAND_ACK, 0u, JETSON_ERROR_BUSY);
        *result = COMMAND_BUSY;
        return NULL;
    }
    entry = &s_command_cache[slot];
    memset(entry, 0, sizeof(*entry));
    entry->used = true;
    entry->type = type;
    entry->seq = seq;
    entry->len = len;
    if ((payload != NULL) && (len > 0u))
    {
        memcpy(entry->payload, payload, len);
    }
    entry->age = ++s_command_cache_age;
    entry->received_ms = now_ms;
    *result = COMMAND_NEW;
    return entry;
}

static void command_ack(command_cache_entry_t *entry, bool accepted, uint8_t error_code)
{
    if (entry == NULL)
    {
        return;
    }
    entry->ack_valid = true;
    entry->ack_value = accepted ? 1u : 0u;
    entry->ack_error = error_code;
    (void)jetson_send_status(entry->seq, RA6_TO_JETSON_COMMAND_ACK,
                             entry->ack_value, error_code);
}

static void command_business(uint8_t type, uint8_t seq, uint8_t event,
                             uint8_t value, uint8_t error_code)
{
    command_cache_entry_t *entry = command_find(type, seq);
    if (entry != NULL)
    {
        entry->business_valid = true;
        entry->business_event = event;
        entry->business_value = value;
        entry->business_error = error_code;
    }
    (void)jetson_send_status(seq, event, value, error_code);
}

static bool workflow_time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t)(now_ms - deadline_ms) >= 0);
}

static void workflow_status(uint8_t value, uint8_t error_code)
{
    command_business(JETSON_MSG_WORKFLOW_CTRL, s_workflow.workflow_seq,
                     RA6_TO_JETSON_WORKFLOW, value, error_code);
}

static void workflow_target_status(uint8_t event, uint8_t value, uint8_t error_code)
{
    command_business(JETSON_MSG_TARGET_CTRL, s_workflow.target_seq,
                     event, value, error_code);
}

static void workflow_force_laser_off(uint8_t error_code)
{
    BSP_Laser_Off();
    if (s_workflow.laser_active)
    {
        s_workflow.laser_active = false;
        workflow_target_status(RA6_TO_JETSON_OUTPUT, 0u, error_code);
    }
    workflow_update_target_indicators();
}

static void workflow_clear_round(void)
{
    s_workflow.distance_safe_latched = false;
    s_workflow.selected_view_id = 0u;
    s_workflow.capture_next_point = 1u;
    s_workflow.capture_done_mask = 0u;
    workflow_clear_safe_samples();
    s_workflow.retreat_steps = 0u;
    s_workflow.retreat_phase = RETREAT_PHASE_IDLE;
    s_workflow.target_enabled = false;
    s_workflow.target_aligned = false;
    workflow_update_target_indicators();
}

static void workflow_stop_finish(uint8_t error_code, uint32_t now_ms)
{
    /* Complete every coalesced ABORT with its own SEQ; never infer success from ACK. */
    for (uint8_t i = 0u; i < WORKFLOW_COMMAND_CACHE_SIZE; i++)
    {
        command_cache_entry_t *entry = &s_command_cache[i];
        if (command_stop_pending(entry))
        {
            command_business(entry->type, entry->seq, RA6_TO_JETSON_WORKFLOW,
                             (error_code == JETSON_ERROR_NONE) ?
                             JETSON_WORKFLOW_ABORTED_HOLD : JETSON_WORKFLOW_FAULT_HOLD,
                             error_code);
            entry->received_ms = now_ms;
        }
    }
    s_stop.pending = false;
    s_stop.restart_locked = (error_code != JETSON_ERROR_NONE);
    s_workflow.fault_latched = s_stop.restart_locked;
    workflow_force_laser_off(error_code);
    if (s_stop.restart_locked)
    {
        scan_invalidate(JETSON_SCAN_FAULT_HOLD, error_code);
        workflow_enter(FLOW_FAULT_HOLD, now_ms);
        (void)jetson_send_error(s_workflow.workflow_seq, error_code);
        LOG("[WORKFLOW] stop unconfirmed error=0x%02X\r\n", error_code);
    }
    else
    {
        LOG("[WORKFLOW] stop confirmed by fresh J1-J5 feedback\r\n");
    }
}

static void workflow_stop_step(uint32_t now_ms)
{
    if (!s_stop.pending)
    {
        return;
    }
    if (workflow_time_reached(now_ms, s_stop.deadline))
    {
        workflow_stop_finish(JETSON_ERROR_MOTION_FAILED, now_ms);
        return;
    }
    if (!workflow_time_reached(now_ms, s_stop.next_sample))
    {
        return;
    }

    float angles[ROBOT_ARM_JOINT_NUM];
    uint32_t sample_ms = now_ms;
    bool valid = robot_motion_stop_feedback(angles, &sample_ms);
    if (workflow_time_reached(sample_ms, s_stop.deadline))
    {
        workflow_stop_finish(JETSON_ERROR_MOTION_FAILED, sample_ms);
        return;
    }
    s_stop.next_sample = sample_ms + ROBOT_WORKFLOW_STOP_SAMPLE_MS;
    bool stable = valid && (s_stop.stable_count != 0u);
    for (uint8_t i = 0u; valid && (i < ROBOT_ARM_JOINT_NUM); i++)
    {
        if (!isfinite(angles[i]))
        {
            valid = false;
            stable = false;
            break;
        }
        float delta = fabsf(angles[i] - s_stop.anchor[i]);
        if (delta > 180.0f) { delta = 360.0f - delta; }
        if (delta > ROBOT_WORKFLOW_STOP_TOL_DEG) { stable = false; }
    }
    if (!valid)
    {
        s_stop.stable_count = 0u;
        return;
    }
    if (!stable)
    {
        memcpy(s_stop.anchor, angles, sizeof(angles));
        s_stop.anchor_ms = sample_ms;
        s_stop.stable_count = 1u;
        return;
    }
    if (s_stop.stable_count < ROBOT_WORKFLOW_STOP_STABLE_COUNT)
    {
        s_stop.stable_count++;
    }
    if ((s_stop.stable_count >= ROBOT_WORKFLOW_STOP_STABLE_COUNT) &&
        ((uint32_t)(sample_ms - s_stop.anchor_ms) >= ROBOT_WORKFLOW_STOP_STABLE_MS))
    {
        workflow_stop_finish(JETSON_ERROR_NONE, sample_ms);
    }
}

static void workflow_fault(uint8_t error_code, uint32_t now_ms)
{
    bool stop_was_pending = s_stop.pending;
    if (s_stop.pending)
    {
        workflow_stop_finish(error_code, now_ms);
    }
    s_stop.restart_locked = true;
    if ((s_workflow.state == FLOW_FAULT_HOLD) && !stop_was_pending)
    {
        workflow_force_laser_off(error_code);
        return;
    }

    workflow_force_laser_off(error_code);
    robot_target_stop_hold();
    if (robot_capture_is_active())
    {
        robot_capture_cancel();
    }
    robot_motion_abort();
    scan_invalidate(JETSON_SCAN_FAULT_HOLD, error_code);
    workflow_clear_round();
    s_workflow.fault_latched = true;
    workflow_enter(FLOW_FAULT_HOLD, now_ms);
    (void)jetson_send_error(0u, error_code);
    workflow_status(JETSON_WORKFLOW_FAULT_HOLD, error_code);
}

static bool workflow_begin_measure_position(uint32_t now_ms)
{
    struct position measure =
    {
        .x = ROBOT_WORKFLOW_MEASURE_X,
        .y = ROBOT_WORKFLOW_MEASURE_Y,
        .z = ROBOT_WORKFLOW_MEASURE_Z,
    };

    if (robot_send_auto_event(&measure) != pdPASS)
    {
        return false;
    }
    workflow_enter(FLOW_MEASURE_POSITION, now_ms);
    s_workflow.state_deadline_ms = now_ms + ROBOT_WORKFLOW_MOTION_TIMEOUT_MS;
    return true;
}

static bool workflow_begin_home(uint32_t now_ms, workflow_state_t state)
{
    if (robot_send_reset_event(false) != pdPASS)
    {
        return false;
    }
    workflow_enter(state, now_ms);
    s_workflow.state_deadline_ms = now_ms + ROBOT_WORKFLOW_RESET_TIMEOUT_MS;
    return true;
}

static bool workflow_begin_retreat_step(uint32_t now_ms)
{
    if (s_workflow.retreat_steps >= ROBOT_WORKFLOW_RETREAT_MAX_STEPS)
    {
        return false;
    }

    struct position retreat = g_robot.cur_pos;
    retreat.y += ROBOT_WORKFLOW_RETREAT_STEP_MM;
    if (robot_send_auto_event(&retreat) != pdPASS)
    {
        return false;
    }

    s_workflow.retreat_steps++;
    s_workflow.retreat_phase = RETREAT_PHASE_MOVING;
    s_workflow.state_deadline_ms = now_ms + ROBOT_WORKFLOW_MOTION_TIMEOUT_MS;
    workflow_enter(FLOW_RETREAT_WAIT_RESTART, now_ms);
    s_workflow.state_deadline_ms = now_ms + ROBOT_WORKFLOW_MOTION_TIMEOUT_MS;
    LOG("[WORKFLOW] retreat step=%u target_y=%.2f\r\n",
        (unsigned)s_workflow.retreat_steps, retreat.y);
    return true;
}

static void workflow_apply_laser_gate(const robot_workflow_obs_t *obs)
{
    bool output_permitted = (obs != NULL) &&
                            (s_workflow.state == FLOW_TARGET_ACTIVE) &&
                            s_workflow.target_enabled &&
                            s_workflow.distance_safe_latched &&
                            !s_workflow.fault_latched &&
                            obs->heartbeat_alive && obs->fire_button &&
                            !obs->estop_active && !obs->limit_triggered &&
                            !workflow_any_limit_triggered() &&
                            ROBOT_STATUS_IS(g_robot.status, ROBOT_STATUS_POSE_VALID) &&
                            !robot_motion_abort_latched() &&
                            !robot_is_auto_busy() &&
                            robot_target_output_requested();

    if (!output_permitted)
    {
        workflow_force_laser_off(JETSON_ERROR_NONE);
        return;
    }

    if (!s_workflow.laser_active)
    {
        taskENTER_CRITICAL();
        bool state_still_valid = (s_workflow.state == FLOW_TARGET_ACTIVE) &&
                                 s_workflow.target_enabled && obs->fire_button &&
                                 !s_workflow.fault_latched;
        if (state_still_valid)
        {
            BSP_Laser_On();
            s_workflow.laser_active = true;
        }
        taskEXIT_CRITICAL();

        if (s_workflow.laser_active)
        {
            workflow_update_target_indicators();
            workflow_target_status(RA6_TO_JETSON_OUTPUT, 1u, JETSON_ERROR_NONE);
        }
    }
}

static uint8_t workflow_target_start_error(const robot_workflow_obs_t *obs)
{
    if ((obs == NULL) || !obs->heartbeat_alive)
    {
        return JETSON_ERROR_HEARTBEAT_TIMEOUT;
    }
    if (obs->estop_active || obs->limit_triggered || workflow_any_limit_triggered())
    {
        return JETSON_ERROR_SAFETY;
    }
    if (!s_workflow.distance_safe_latched || (s_workflow.selected_view_id == 0u))
    {
        return JETSON_ERROR_TARGET_GATE_DENIED;
    }
    if (!ROBOT_STATUS_IS(g_robot.status, ROBOT_STATUS_POSE_VALID))
    {
        return JETSON_ERROR_POSE_INVALID;
    }
    if (s_workflow.fault_latched || robot_motion_abort_latched())
    {
        return JETSON_ERROR_MOTION_ABORTED;
    }
    if (robot_is_auto_busy() || robot_capture_is_active())
    {
        return JETSON_ERROR_BUSY;
    }
    return JETSON_ERROR_NONE;
}

static void workflow_handle_workflow_command(const robot_workflow_obs_t *obs)
{
    uint8_t payload[1] = {obs->workflow_action};
    command_prepare_result_t prepare = COMMAND_NEW;
    command_cache_entry_t *entry = command_prepare(JETSON_MSG_WORKFLOW_CTRL,
                                                    obs->workflow_seq,
                                                    payload, sizeof(payload),
                                                    &prepare, obs->now_ms);
    if (prepare != COMMAND_NEW)
    {
        return;
    }

    switch (obs->workflow_action)
    {
        case JETSON_WORKFLOW_ACTION_START_MEASURE:
            if (s_stop.pending || s_stop.restart_locked)
            {
                command_ack(entry, false, s_stop.pending ? JETSON_ERROR_BUSY :
                            JETSON_ERROR_MOTION_FAILED);
                return;
            }
            if (!obs->heartbeat_alive)
            {
                command_ack(entry, false, JETSON_ERROR_HEARTBEAT_TIMEOUT);
                return;
            }
            if (obs->estop_active || obs->limit_triggered || workflow_any_limit_triggered())
            {
                command_ack(entry, false, JETSON_ERROR_SAFETY);
                return;
            }
            if ((s_workflow.state != FLOW_IDLE) &&
                (s_workflow.state != FLOW_ABORT_HOLD) &&
                (s_workflow.state != FLOW_FAULT_HOLD))
            {
                if ((s_workflow.state != FLOW_RETREAT_WAIT_RESTART) ||
                    (s_workflow.retreat_phase != RETREAT_PHASE_DONE))
                {
                    command_ack(entry, false, JETSON_ERROR_INVALID_STATE);
                    return;
                }
            }

            workflow_force_laser_off(JETSON_ERROR_NONE);
            robot_target_stop_hold();
            if (robot_capture_is_active())
            {
                robot_capture_cancel();
            }
            workflow_clear_round();
            s_workflow.fault_latched = false;
            s_workflow.workflow_seq = obs->workflow_seq;
            if (!workflow_begin_home(obs->now_ms, FLOW_HOMING))
            {
                command_ack(entry, false, JETSON_ERROR_BUSY);
                workflow_fault(JETSON_ERROR_MOTION_FAILED, obs->now_ms);
                return;
            }
            command_ack(entry, true, JETSON_ERROR_NONE);
            workflow_status(JETSON_WORKFLOW_START_ACCEPTED, JETSON_ERROR_NONE);
            break;

        case JETSON_WORKFLOW_ACTION_FINISH_RETURN_HOME:
            if ((s_workflow.state != FLOW_SELECTED_VIEW) &&
                (s_workflow.state != FLOW_TARGET_ACTIVE) &&
                (s_workflow.state != FLOW_TARGET_HOLD) &&
                (s_workflow.state != FLOW_SCAN_READY))
            {
                command_ack(entry, false, JETSON_ERROR_INVALID_STATE);
                return;
            }
            workflow_force_laser_off(JETSON_ERROR_NONE);
            robot_target_stop_hold();
            if (robot_is_auto_busy())
            {
                robot_motion_abort();
            }
            workflow_clear_round();
            if (s_scan.state == JETSON_SCAN_FRONT_OBSERVE_HOLD)
            {
                s_scan.state = JETSON_SCAN_CHAIN_COMPLETED;
                s_scan.flags = 0u;
            }
            s_workflow.workflow_seq = obs->workflow_seq;
            if (!workflow_begin_home(obs->now_ms, FLOW_RETURN_HOME))
            {
                command_ack(entry, false, JETSON_ERROR_BUSY);
                workflow_fault(JETSON_ERROR_MOTION_FAILED, obs->now_ms);
                return;
            }
            command_ack(entry, true, JETSON_ERROR_NONE);
            break;

        case JETSON_WORKFLOW_ACTION_ABORT_HOLD:
            if (s_stop.pending)
            {
                command_ack(entry, true, JETSON_ERROR_NONE);
                break;
            }
            s_workflow.workflow_seq = obs->workflow_seq;
            workflow_force_laser_off(JETSON_ERROR_NONE);
            robot_target_stop_hold();
            if (robot_capture_is_active())
            {
                robot_capture_cancel();
            }
            s_stop.pending = true;
            s_stop.restart_locked = true;
            s_stop.stable_count = 0u;
            s_stop.deadline = obs->now_ms + ROBOT_WORKFLOW_STOP_TIMEOUT_MS;
            s_stop.next_sample = obs->now_ms;
            bool stop_acknowledged = robot_motion_abort();
            scan_invalidate(JETSON_SCAN_ABORTED_HOLD, JETSON_ERROR_MOTION_ABORTED);
            workflow_clear_round();
            s_workflow.fault_latched = false;
            workflow_enter(FLOW_ABORT_HOLD, obs->now_ms);
            command_ack(entry, true, JETSON_ERROR_NONE);
            if (!stop_acknowledged)
            {
                workflow_stop_finish(JETSON_ERROR_MOTION_FAILED, obs->now_ms);
            }
            break;

        default:
            command_ack(entry, false, JETSON_ERROR_INVALID_PARAM);
            break;
    }
}

static bool workflow_handle_target_command(const robot_workflow_obs_t *obs)
{
    uint8_t payload[1] = {obs->target_value};
    command_prepare_result_t prepare = COMMAND_NEW;
    command_cache_entry_t *entry = command_prepare(JETSON_MSG_TARGET_CTRL,
                                                    obs->target_seq,
                                                    payload, sizeof(payload),
                                                    &prepare, obs->now_ms);
    if (prepare != COMMAND_NEW)
    {
        return false;
    }

    if (obs->target_value > 1u)
    {
        command_ack(entry, false, JETSON_ERROR_INVALID_PARAM);
        return false;
    }

    if (obs->target_value == 0u)
    {
        s_workflow.target_aligned = false;
        workflow_force_laser_off(JETSON_ERROR_NONE);
        robot_target_stop_hold();
        s_workflow.target_enabled = false;
        s_workflow.target_seq = obs->target_seq;
        if (s_workflow.state == FLOW_TARGET_ACTIVE)
        {
            workflow_enter(FLOW_TARGET_HOLD, obs->now_ms);
        }
        command_ack(entry, true, JETSON_ERROR_NONE);
        workflow_target_status(RA6_TO_JETSON_TARGET_CTRL, 0u, JETSON_ERROR_NONE);
        return false;
    }

    if ((s_workflow.state != FLOW_SELECTED_VIEW) &&
        (s_workflow.state != FLOW_TARGET_HOLD))
    {
        command_ack(entry, false, JETSON_ERROR_INVALID_STATE);
        command_business(JETSON_MSG_TARGET_CTRL, obs->target_seq,
                         RA6_TO_JETSON_TARGET_CTRL, 0u,
                         JETSON_ERROR_INVALID_STATE);
        return false;
    }

    uint8_t error_code = workflow_target_start_error(obs);
    if (error_code != JETSON_ERROR_NONE)
    {
        command_ack(entry, false, error_code);
        command_business(JETSON_MSG_TARGET_CTRL, obs->target_seq,
                         RA6_TO_JETSON_TARGET_CTRL, 0u, error_code);
        return false;
    }

    if (!robot_target_start_at_current_pose())
    {
        command_ack(entry, false, JETSON_ERROR_MOTION_FAILED);
        command_business(JETSON_MSG_TARGET_CTRL, obs->target_seq,
                         RA6_TO_JETSON_TARGET_CTRL, 0u,
                         JETSON_ERROR_MOTION_FAILED);
        return false;
    }

    s_workflow.target_seq = obs->target_seq;
    s_workflow.target_enabled = true;
    s_workflow.target_aligned = false;
    workflow_enter(FLOW_TARGET_ACTIVE, obs->now_ms);
    command_ack(entry, true, JETSON_ERROR_NONE);
    workflow_target_status(RA6_TO_JETSON_TARGET_CTRL, 1u, JETSON_ERROR_NONE);
    return true;
}

static void workflow_handle_capture_command(const robot_workflow_obs_t *obs)
{
    uint8_t payload[2] = {obs->capture_action, obs->capture_point_id};
    command_prepare_result_t prepare = COMMAND_NEW;
    command_cache_entry_t *entry = command_prepare(JETSON_MSG_CAPTURE_CTRL,
                                                    obs->capture_seq,
                                                    payload, sizeof(payload),
                                                    &prepare, obs->now_ms);
    if (prepare != COMMAND_NEW)
    {
        return;
    }

    bool state_valid = false;
    if ((obs->capture_action == JETSON_CAPTURE_ACTION_HOME) &&
        (obs->capture_point_id == 0u))
    {
        state_valid = (s_workflow.state == FLOW_SAFE_READY) ||
                      ((s_workflow.state == FLOW_CAPTURE) &&
                       (s_workflow.capture_done_mask == 0x07u));
    }
    else if (obs->capture_action == JETSON_CAPTURE_ACTION_GOTO)
    {
        state_valid = (s_workflow.state == FLOW_CAPTURE) &&
                      (obs->capture_point_id == s_workflow.capture_next_point) &&
                      (obs->capture_point_id >= 1u) &&
                      (obs->capture_point_id <= 3u);
    }
    else if (obs->capture_action == JETSON_CAPTURE_ACTION_SELECT)
    {
        state_valid = (s_workflow.state == FLOW_CAPTURE_HOME) &&
                      (s_workflow.capture_done_mask == 0x07u) &&
                      (obs->capture_point_id >= 1u) &&
                      (obs->capture_point_id <= 3u);
    }
    else if ((obs->capture_action != JETSON_CAPTURE_ACTION_HOME) &&
             (obs->capture_action != JETSON_CAPTURE_ACTION_GOTO) &&
             (obs->capture_action != JETSON_CAPTURE_ACTION_SELECT))
    {
        command_ack(entry, false, JETSON_ERROR_INVALID_PARAM);
        return;
    }

    if (!state_valid || !s_workflow.distance_safe_latched)
    {
        command_ack(entry, false, JETSON_ERROR_INVALID_STATE);
        return;
    }
    if (!obs->heartbeat_alive)
    {
        command_ack(entry, false, JETSON_ERROR_HEARTBEAT_TIMEOUT);
        return;
    }
    if (!robot_capture_request(obs->capture_action, obs->capture_point_id))
    {
        command_ack(entry, false, JETSON_ERROR_BUSY);
        return;
    }

    s_workflow.capture_seq = obs->capture_seq;
    s_workflow.capture_action = obs->capture_action;
    s_workflow.capture_point_id = obs->capture_point_id;
    command_ack(entry, true, JETSON_ERROR_NONE);
}

static void workflow_capture_failed(robot_capture_result_t result, uint32_t now_ms)
{
    uint8_t error_code = (result == ROBOT_CAPTURE_RESULT_ABORTED) ?
                         JETSON_ERROR_MOTION_ABORTED : JETSON_ERROR_MOTION_FAILED;
    uint8_t event = (s_workflow.capture_action == JETSON_CAPTURE_ACTION_HOME) ?
                    RA6_TO_JETSON_CAPTURE_HOME :
                    ((s_workflow.capture_action == JETSON_CAPTURE_ACTION_SELECT) ?
                     RA6_TO_JETSON_SELECTED_VIEW : RA6_TO_JETSON_CAPTURE_POINT);
    command_business(JETSON_MSG_CAPTURE_CTRL, s_workflow.capture_seq,
                     event, 0u, error_code);
    workflow_fault(error_code, now_ms);
}

static void workflow_handle_capture_result(uint32_t now_ms)
{
    uint8_t action = 0u;
    uint8_t point_id = 0u;
    robot_capture_result_t result = robot_capture_result_consume(&action, &point_id);
    if ((result == ROBOT_CAPTURE_RESULT_NONE) ||
        (result == ROBOT_CAPTURE_RESULT_RUNNING))
    {
        return;
    }
    if (s_workflow.state == FLOW_SCAN_ACTIVE)
    {
        if (workflow_time_reached(now_ms, s_scan.deadline))
        {
            workflow_fault(JETSON_ERROR_SCAN_STAGE_TIMEOUT, now_ms);
            return;
        }
        if (result != ROBOT_CAPTURE_RESULT_OK)
        {
            workflow_fault((result == ROBOT_CAPTURE_RESULT_ABORTED) ?
                JETSON_ERROR_MOTION_ABORTED : JETSON_ERROR_MOTION_FAILED, now_ms);
            return;
        }
        if ((s_scan.state == JETSON_SCAN_PREPARING) &&
            (action == JETSON_CAPTURE_ACTION_GOTO) && (point_id == 1u))
        {
            if (robot_send_scan_segment_event(ROBOT_SCAN_LEFT_TO_FRONT) != pdPASS)
            {
                workflow_fault(JETSON_ERROR_MOTION_FAILED, now_ms);
                return;
            }
            s_scan.state = JETSON_SCAN_LEFT_TO_FRONT;
            s_scan.deadline = now_ms + ROBOT_SCAN_SEGMENT_TIMEOUT_MS;
        }
        else if ((s_scan.state == JETSON_SCAN_RETURNING_HOME) &&
                 (action == JETSON_CAPTURE_ACTION_HOME) && (point_id == 0u))
        {
            if (!robot_scan_verify_pose(0u))
            {
                workflow_fault(JETSON_ERROR_POSE_INVALID, now_ms);
                return;
            }
            s_scan.flags |= JETSON_SCAN_HOME_VERIFIED;
            s_scan.state = JETSON_SCAN_RETURNING_FRONT;
            s_scan.deadline = now_ms + ROBOT_SCAN_PREPARE_TIMEOUT_MS;
            scan_report(s_scan.start_seq, JETSON_SCAN_HOME_READY,
                        JETSON_SCAN_START, 1u, JETSON_ERROR_NONE);
            if (!robot_capture_request(JETSON_CAPTURE_ACTION_GOTO, 2u))
            {
                workflow_fault(JETSON_ERROR_MOTION_FAILED, now_ms);
            }
        }
        else if ((s_scan.state == JETSON_SCAN_RETURNING_FRONT) &&
                 (action == JETSON_CAPTURE_ACTION_GOTO) && (point_id == 2u))
        {
            if (!robot_scan_verify_pose(2u))
            {
                workflow_fault(JETSON_ERROR_POSE_INVALID, now_ms);
                return;
            }
            s_scan.state = JETSON_SCAN_FRONT_OBSERVE_HOLD;
            workflow_enter(FLOW_SCAN_READY, now_ms);
            /* No legacy SELECTED_VIEW: target identity/doctor permit not wired yet. */
            scan_report(s_scan.start_seq, JETSON_SCAN_FRONT_OBSERVE_READY,
                        JETSON_SCAN_START, 1u, JETSON_ERROR_NONE);
        }
        else
        {
            workflow_fault(JETSON_ERROR_INVALID_STATE, now_ms);
        }
        return;
    }
    if (result != ROBOT_CAPTURE_RESULT_OK)
    {
        workflow_capture_failed(result, now_ms);
        return;
    }

    if ((action == JETSON_CAPTURE_ACTION_HOME) &&
        (s_workflow.state == FLOW_SAFE_READY))
    {
        workflow_enter(FLOW_CAPTURE, now_ms);
        s_workflow.capture_next_point = 1u;
        command_business(JETSON_MSG_CAPTURE_CTRL, s_workflow.capture_seq,
                         RA6_TO_JETSON_CAPTURE_HOME, 1u, JETSON_ERROR_NONE);
    }
    else if ((action == JETSON_CAPTURE_ACTION_GOTO) &&
             (s_workflow.state == FLOW_CAPTURE) &&
             (point_id == s_workflow.capture_next_point))
    {
        s_workflow.capture_done_mask |= (uint8_t)(1u << (point_id - 1u));
        s_workflow.capture_next_point++;
        command_business(JETSON_MSG_CAPTURE_CTRL, s_workflow.capture_seq,
                         RA6_TO_JETSON_CAPTURE_POINT, point_id,
                         JETSON_ERROR_NONE);
    }
    else if ((action == JETSON_CAPTURE_ACTION_HOME) &&
             (s_workflow.state == FLOW_CAPTURE) &&
             (s_workflow.capture_done_mask == 0x07u))
    {
        workflow_enter(FLOW_CAPTURE_HOME, now_ms);
        command_business(JETSON_MSG_CAPTURE_CTRL, s_workflow.capture_seq,
                         RA6_TO_JETSON_CAPTURE_HOME, 1u, JETSON_ERROR_NONE);
    }
    else if ((action == JETSON_CAPTURE_ACTION_SELECT) &&
             (s_workflow.state == FLOW_CAPTURE_HOME))
    {
        s_workflow.selected_view_id = point_id;
        workflow_enter(FLOW_SELECTED_VIEW, now_ms);
        command_business(JETSON_MSG_CAPTURE_CTRL, s_workflow.capture_seq,
                         RA6_TO_JETSON_SELECTED_VIEW, point_id,
                         JETSON_ERROR_NONE);
    }
    else
    {
        workflow_fault(JETSON_ERROR_INVALID_STATE, now_ms);
    }
}

static void workflow_handle_reset_and_motion_results(uint32_t now_ms)
{
    if ((s_workflow.state == FLOW_HOMING) ||
        (s_workflow.state == FLOW_RETURN_HOME))
    {
        robot_reset_result_t result = robot_reset_result_consume();
        if (result == ROBOT_RESET_RESULT_OK)
        {
            if (s_workflow.state == FLOW_HOMING)
            {
                if (!workflow_begin_measure_position(now_ms))
                {
                    workflow_fault(JETSON_ERROR_MOTION_FAILED, now_ms);
                }
            }
            else
            {
                workflow_clear_round();
                s_workflow.fault_latched = false;
                workflow_enter(FLOW_IDLE, now_ms);
                workflow_status(JETSON_WORKFLOW_RETURN_HOME_DONE, JETSON_ERROR_NONE);
            }
        }
        else if ((result == ROBOT_RESET_RESULT_FAILED) ||
                 (result == ROBOT_RESET_RESULT_ABORTED))
        {
            workflow_fault((result == ROBOT_RESET_RESULT_ABORTED) ?
                           JETSON_ERROR_MOTION_ABORTED :
                           JETSON_ERROR_SOFT_RESET_FAILED, now_ms);
        }
        else if (workflow_time_reached(now_ms, s_workflow.state_deadline_ms))
        {
            workflow_fault(JETSON_ERROR_SOFT_RESET_FAILED, now_ms);
        }
        return;
    }

    if (s_workflow.state == FLOW_MEASURE_POSITION)
    {
        robot_auto_result_t result = robot_auto_result_consume();
        if (result == ROBOT_AUTO_RESULT_OK)
        {
            workflow_clear_safe_samples();
            workflow_enter(FLOW_WAIT_SAFE_DISTANCE, now_ms);
            workflow_status(JETSON_WORKFLOW_MEASURE_POSITION_READY,
                            JETSON_ERROR_NONE);
        }
        else if ((result == ROBOT_AUTO_RESULT_FAILED) ||
                 (result == ROBOT_AUTO_RESULT_ABORTED))
        {
            workflow_fault((result == ROBOT_AUTO_RESULT_ABORTED) ?
                           JETSON_ERROR_MOTION_ABORTED :
                           JETSON_ERROR_MOTION_FAILED, now_ms);
        }
        else if (workflow_time_reached(now_ms, s_workflow.state_deadline_ms))
        {
            workflow_fault(JETSON_ERROR_MOTION_FAILED, now_ms);
        }
    }
}

static void workflow_handle_safe_distance(const robot_workflow_obs_t *obs)
{
    if (!obs->has_distance)
    {
        return;
    }

    if (obs->distance_seq == 0u)
    {
        workflow_clear_safe_samples();
        LOG("[SAFE] zero seq rejected\r\n");
        (void)jetson_send_error(0u, JETSON_ERROR_INVALID_PARAM);
        return;
    }

    if (s_workflow.state == FLOW_WAIT_SAFE_DISTANCE)
    {
        if (!obs->distance_valid)
        {
            workflow_clear_safe_samples();
            LOG("[SAFE] seq=%u invalid, streak cleared\r\n",
                (unsigned)obs->distance_seq);
            return;
        }

        if (obs->distance_mm >= ROBOT_WORKFLOW_SAFE_DISTANCE_MM)
        {
            if (workflow_safe_seq_seen(obs->distance_seq))
            {
                LOG("[SAFE] duplicate seq=%u ignored streak=%u\r\n",
                    (unsigned)obs->distance_seq,
                    (unsigned)s_workflow.safe_sample_count);
                return;
            }

            if (s_workflow.safe_sample_count < ROBOT_WORKFLOW_SAFE_STABLE_COUNT)
            {
                s_workflow.safe_sample_seqs[s_workflow.safe_sample_count] =
                    obs->distance_seq;
                s_workflow.safe_sample_count++;
            }
            LOG("[SAFE] seq=%u mm=%u streak=%u/%u\r\n",
                (unsigned)obs->distance_seq,
                (unsigned)obs->distance_mm,
                (unsigned)s_workflow.safe_sample_count,
                (unsigned)ROBOT_WORKFLOW_SAFE_STABLE_COUNT);

            if (s_workflow.safe_sample_count >= ROBOT_WORKFLOW_SAFE_STABLE_COUNT)
            {
                s_workflow.distance_safe_latched = true;
                (void)jetson_send_status(obs->distance_seq,
                                         RA6_TO_JETSON_SAFE_DISTANCE, 1u,
                                         JETSON_ERROR_NONE);
                workflow_enter(FLOW_SAFE_READY, obs->now_ms);
                workflow_status(JETSON_WORKFLOW_DISTANCE_SAFE_LATCHED,
                                JETSON_ERROR_NONE);
            }
            return;
        }

        workflow_clear_safe_samples();
        s_workflow.distance_safe_latched = false;
        (void)jetson_send_status(obs->distance_seq,
                                 RA6_TO_JETSON_SAFE_DISTANCE, 0u,
                                 JETSON_ERROR_SAFE_DISTANCE_TOO_CLOSE);
        workflow_force_laser_off(JETSON_ERROR_SAFE_DISTANCE_TOO_CLOSE);
        if (!workflow_begin_retreat_step(obs->now_ms))
        {
            workflow_fault(JETSON_ERROR_SAFE_DISTANCE_TOO_CLOSE, obs->now_ms);
        }
        return;
    }

    if ((s_workflow.state != FLOW_RETREAT_WAIT_RESTART) ||
        (s_workflow.retreat_phase != RETREAT_PHASE_WAIT_DISTANCE))
    {
        return;
    }
    if (!obs->distance_valid)
    {
        LOG("[SAFE] retreat measurement seq=%u invalid\r\n",
            (unsigned)obs->distance_seq);
        return;
    }

    if (obs->distance_mm >= ROBOT_WORKFLOW_SAFE_DISTANCE_MM)
    {
        (void)jetson_send_status(obs->distance_seq,
                                 RA6_TO_JETSON_SAFE_DISTANCE, 1u,
                                 JETSON_ERROR_NONE);
        s_workflow.retreat_phase = RETREAT_PHASE_DONE;
        workflow_status(JETSON_WORKFLOW_RETREAT_DONE_WAIT_RESTART,
                        JETSON_ERROR_NONE);
        LOG("[SAFE] retreat complete steps=%u mm=%u, wait restart\r\n",
            (unsigned)s_workflow.retreat_steps,
            (unsigned)obs->distance_mm);
        return;
    }

    (void)jetson_send_status(obs->distance_seq,
                             RA6_TO_JETSON_SAFE_DISTANCE, 0u,
                             JETSON_ERROR_SAFE_DISTANCE_TOO_CLOSE);
    LOG("[SAFE] retreat still close step=%u mm=%u\r\n",
        (unsigned)s_workflow.retreat_steps,
        (unsigned)obs->distance_mm);
    if (!workflow_begin_retreat_step(obs->now_ms))
    {
        workflow_fault(JETSON_ERROR_SAFE_DISTANCE_TOO_CLOSE, obs->now_ms);
    }
}
static void workflow_handle_retreat_motion(uint32_t now_ms)
{
    if ((s_workflow.state != FLOW_RETREAT_WAIT_RESTART) ||
        (s_workflow.retreat_phase == RETREAT_PHASE_DONE))
    {
        return;
    }

    if (s_workflow.retreat_phase == RETREAT_PHASE_MOVING)
    {
        robot_auto_result_t result = robot_auto_result_consume();
        if (result == ROBOT_AUTO_RESULT_OK)
        {
            s_workflow.retreat_phase = RETREAT_PHASE_WAIT_DISTANCE;
            s_workflow.state_deadline_ms = now_ms + ROBOT_WORKFLOW_RETREAT_WAIT_MS;
            workflow_status(JETSON_WORKFLOW_RETREAT_STEP_READY,
                            JETSON_ERROR_NONE);
        }
        else if ((result == ROBOT_AUTO_RESULT_FAILED) ||
                 (result == ROBOT_AUTO_RESULT_ABORTED) ||
                 workflow_time_reached(now_ms, s_workflow.state_deadline_ms))
        {
            workflow_fault((result == ROBOT_AUTO_RESULT_ABORTED) ?
                           JETSON_ERROR_MOTION_ABORTED :
                           JETSON_ERROR_MOTION_FAILED, now_ms);
        }
    }
    else if ((s_workflow.retreat_phase == RETREAT_PHASE_WAIT_DISTANCE) &&
             workflow_time_reached(now_ms, s_workflow.state_deadline_ms))
    {
        workflow_fault(JETSON_ERROR_SAFETY, now_ms);
    }
}

static void workflow_handle_target_event(uint32_t now_ms)
{
    robot_target_event_t event = robot_target_event_consume();
    switch (event)
    {
        case ROBOT_TARGET_EVENT_READY:
            workflow_target_status(RA6_TO_JETSON_READY, 1u,
                                   JETSON_ERROR_NONE);
            break;

        case ROBOT_TARGET_EVENT_VISION_RECOVERED:
            workflow_target_status(RA6_TO_JETSON_VISION_STATE, 1u,
                                   JETSON_ERROR_NONE);
            break;

        case ROBOT_TARGET_EVENT_ALIGN_DONE:
            s_workflow.target_aligned = true;
            workflow_update_target_indicators();
            workflow_target_status(RA6_TO_JETSON_ALIGN_DONE, 1u,
                                   JETSON_ERROR_NONE);
            break;

        case ROBOT_TARGET_EVENT_VISION_LOST:
            s_workflow.target_aligned = false;
            workflow_force_laser_off(JETSON_ERROR_VISION_LOST);
            workflow_target_status(RA6_TO_JETSON_VISION_STATE, 0u,
                                   JETSON_ERROR_VISION_LOST);
            break;

        case ROBOT_TARGET_EVENT_ALIGNMENT_LOST:
            s_workflow.target_aligned = false;
            workflow_force_laser_off(JETSON_ERROR_NONE);
            workflow_target_status(RA6_TO_JETSON_ALIGN_DONE, 0u,
                                   JETSON_ERROR_NONE);
            break;

        case ROBOT_TARGET_EVENT_HOLD:
            s_workflow.target_aligned = false;
            workflow_force_laser_off(JETSON_ERROR_NONE);
            workflow_enter(FLOW_TARGET_HOLD, now_ms);
            break;

        case ROBOT_TARGET_EVENT_FAULT:
            workflow_fault(JETSON_ERROR_MOTION_FAILED, now_ms);
            break;

        case ROBOT_TARGET_EVENT_NONE:
        default:
            break;
    }
}

static void workflow_scan_command(const robot_workflow_obs_t *obs)
{
    const jetson_scan_control_t *c = &obs->scan_control;
    uint8_t error = JETSON_ERROR_NONE;
    if ((c->seq == 0u) || (c->action > JETSON_SCAN_PHOTO_DONE) ||
        ((c->action != JETSON_SCAN_PHOTO_DONE) && (c->value != 0u)) ||
        ((c->action == JETSON_SCAN_PHOTO_DONE) && (c->value > 1u)) ||
        ((c->action != JETSON_SCAN_QUERY) && (c->scan_id == 0u)))
    {
        error = JETSON_ERROR_INVALID_PARAM;
    }
    else if (c->action == JETSON_SCAN_QUERY)
    {
        if ((c->scan_id != 0u) && (c->scan_id != s_scan.id))
        {
            error = JETSON_ERROR_SCAN_ID_MISMATCH;
        }
        else
        {
            if (((s_workflow.state == FLOW_SCAN_READY) ||
                 ((s_workflow.state == FLOW_SCAN_ACTIVE) && (s_scan.state == JETSON_SCAN_WAIT_PHOTO))) &&
                !robot_scan_verify_pose(2u))
            {
                workflow_fault(JETSON_ERROR_POSE_INVALID, obs->now_ms);
            }
            scan_report(c->seq, JETSON_SCAN_ACK, c->action, 1u, JETSON_ERROR_NONE);
            scan_snapshot(c->seq, c->action);
            return;
        }
    }
    else if ((c->scan_id == s_scan.id) &&
             (((c->seq == s_scan.start_seq) && (c->action != JETSON_SCAN_START)) ||
              (s_scan.photo_seen && (c->seq == s_scan.photo_seq) &&
               ((c->action != JETSON_SCAN_PHOTO_DONE) || (c->value != s_scan.photo_value)))))
    {
        error = JETSON_ERROR_SEQ_CONFLICT;
    }
    else if ((c->action == JETSON_SCAN_START) && (c->scan_id == s_scan.id))
    {
        scan_report(c->seq, JETSON_SCAN_ACK, c->action, 1u, JETSON_ERROR_NONE);
        scan_snapshot(c->seq, c->action);
        return;
    }
    else if (c->action == JETSON_SCAN_START)
    {
        if (c->scan_id <= s_scan.high_water)
        {
            error = JETSON_ERROR_SCAN_ID_MISMATCH;
        }
        else if ((s_workflow.state != FLOW_SAFE_READY) || !s_workflow.distance_safe_latched)
        {
            error = JETSON_ERROR_INVALID_STATE;
        }
        else if (!obs->heartbeat_alive)
        {
            error = JETSON_ERROR_HEARTBEAT_TIMEOUT;
        }
        else if (robot_capture_is_active() || robot_is_auto_busy() || robot_motion_abort_latched())
        {
            error = JETSON_ERROR_BUSY;
        }
        else
        {
            workflow_force_laser_off(JETSON_ERROR_NONE);
            robot_target_stop_hold();
            /* A previous legacy round may have cached the pre-HOME base pose. */
            robot_capture_init();
            if (!robot_capture_request(JETSON_CAPTURE_ACTION_GOTO, 1u))
            {
                error = JETSON_ERROR_BUSY;
            }
            else
            {
                memset(&s_scan, 0, sizeof(s_scan));
                s_scan.id = s_scan.high_water = c->scan_id;
                s_scan.start_seq = c->seq;
                s_scan.state = JETSON_SCAN_PREPARING;
                s_scan.deadline = obs->now_ms + ROBOT_SCAN_PREPARE_TIMEOUT_MS;
                s_workflow.selected_view_id = 0u;
                workflow_enter(FLOW_SCAN_ACTIVE, obs->now_ms);
                scan_report(c->seq, JETSON_SCAN_ACK, c->action, 1u, JETSON_ERROR_NONE);
                return;
            }
        }
    }
    else if (c->scan_id != s_scan.id)
    {
        error = JETSON_ERROR_SCAN_ID_MISMATCH;
    }
    else if (s_scan.photo_seen)
    {
        if (c->value != s_scan.photo_value)
        {
            error = JETSON_ERROR_SEQ_CONFLICT;
        }
        else
        {
            scan_report(c->seq, JETSON_SCAN_ACK, c->action, 1u, JETSON_ERROR_NONE);
            scan_snapshot(c->seq, c->action);
            return;
        }
    }
    else if ((s_workflow.state != FLOW_SCAN_ACTIVE) || (s_scan.state != JETSON_SCAN_WAIT_PHOTO))
    {
        error = JETSON_ERROR_INVALID_STATE;
    }
    else if (workflow_time_reached(obs->now_ms, s_scan.deadline))
    {
        error = JETSON_ERROR_PHOTO_CONFIRM_TIMEOUT;
        workflow_fault(error, obs->now_ms);
    }
    else
    {
        s_scan.photo_seen = true;
        s_scan.photo_seq = c->seq;
        s_scan.photo_value = c->value;
        scan_report(c->seq, JETSON_SCAN_ACK, c->action, 1u, JETSON_ERROR_NONE);
        if (c->value == 0u)
        {
            workflow_fault(JETSON_ERROR_PHOTO_SAVE_FAILED, obs->now_ms);
            return;
        }
        if (robot_send_scan_segment_event(ROBOT_SCAN_FRONT_TO_RIGHT) != pdPASS)
        {
            workflow_fault(JETSON_ERROR_MOTION_FAILED, obs->now_ms);
            return;
        }
        s_scan.flags = JETSON_SCAN_PHOTO_SAVED;
        s_scan.state = JETSON_SCAN_FRONT_TO_RIGHT;
        s_scan.deadline = obs->now_ms + ROBOT_SCAN_SEGMENT_TIMEOUT_MS;
        scan_report(c->seq, JETSON_SCAN_PHOTO_ACCEPTED, c->action, 1u, JETSON_ERROR_NONE);
        return;
    }
    /* Rejected requests echo their own ID, never another round's ready flags. */
    (void)jetson_send_scan_status(c->seq, JETSON_SCAN_ACK, JETSON_SCAN_NONE,
        0u, c->action, 0u, error, c->scan_id);
}

static void workflow_scan_step(uint32_t now_ms)
{
    if (s_workflow.state == FLOW_SCAN_READY)
    {
        if (robot_is_auto_busy() || !ROBOT_STATUS_IS(g_robot.status, ROBOT_STATUS_POSE_VALID) ||
            (workflow_time_reached(now_ms, s_scan.next_verify) && !robot_scan_verify_pose(2u)))
        {
            workflow_fault(JETSON_ERROR_POSE_INVALID, now_ms);
        }
        if (workflow_time_reached(now_ms, s_scan.next_verify))
        {
            s_scan.next_verify = now_ms + 300u;
        }
        return;
    }
    if (s_workflow.state != FLOW_SCAN_ACTIVE)
    {
        return;
    }
    if (workflow_time_reached(now_ms, s_scan.deadline))
    {
        workflow_fault((s_scan.state == JETSON_SCAN_WAIT_PHOTO) ?
            JETSON_ERROR_PHOTO_CONFIRM_TIMEOUT : JETSON_ERROR_SCAN_STAGE_TIMEOUT, now_ms);
        return;
    }
    if ((s_scan.state == JETSON_SCAN_WAIT_PHOTO) && workflow_time_reached(now_ms, s_scan.next_verify))
    {
        s_scan.next_verify = now_ms + 300u;
        if (!robot_scan_verify_pose(2u))
        {
            workflow_fault(JETSON_ERROR_POSE_INVALID, now_ms);
        }
        return;
    }
    if ((s_scan.state != JETSON_SCAN_LEFT_TO_FRONT) &&
        (s_scan.state != JETSON_SCAN_FRONT_TO_RIGHT))
    {
        return;
    }
    robot_scan_result_t result = robot_scan_result_get();
    if ((result == ROBOT_SCAN_RESULT_FAILED) || (result == ROBOT_SCAN_RESULT_ABORTED))
    {
        workflow_fault((result == ROBOT_SCAN_RESULT_ABORTED) ?
            JETSON_ERROR_MOTION_ABORTED : JETSON_ERROR_MOTION_FAILED, now_ms);
    }
    else if ((s_scan.state == JETSON_SCAN_LEFT_TO_FRONT) &&
             (result == ROBOT_SCAN_RESULT_FRONT_READY))
    {
        s_scan.state = JETSON_SCAN_WAIT_PHOTO;
        s_scan.deadline = now_ms + ROBOT_SCAN_PHOTO_TIMEOUT_MS;
        s_scan.next_verify = now_ms + 300u;
        scan_report(s_scan.start_seq, JETSON_SCAN_FRONT_PHOTO_READY,
                    JETSON_SCAN_START, 1u, JETSON_ERROR_NONE);
    }
    else if ((s_scan.state == JETSON_SCAN_FRONT_TO_RIGHT) &&
             (result == ROBOT_SCAN_RESULT_RIGHT_READY))
    {
        s_scan.flags |= JETSON_SCAN_FINISHED;
        s_scan.state = JETSON_SCAN_RETURNING_HOME;
        s_scan.deadline = now_ms + ROBOT_WORKFLOW_RESET_TIMEOUT_MS;
        scan_report(s_scan.start_seq, JETSON_SCAN_DONE, JETSON_SCAN_START, 1u, JETSON_ERROR_NONE);
        if (!robot_capture_request(JETSON_CAPTURE_ACTION_HOME, 0u))
        {
            workflow_fault(JETSON_ERROR_MOTION_FAILED, now_ms);
        }
    }
}

static void workflow_reject_pending_commands(const robot_workflow_obs_t *obs,
                                             uint8_t error_code)
{
    command_prepare_result_t prepare = COMMAND_NEW;
    command_cache_entry_t *entry = NULL;

    if (obs->has_scan_control)
    {
        if (obs->scan_control.action == JETSON_SCAN_QUERY)
        {
            workflow_scan_command(obs);
        }
        else
        {
            (void)jetson_send_scan_status(obs->scan_control.seq, JETSON_SCAN_ACK,
                JETSON_SCAN_NONE, 0u, obs->scan_control.action, 0u,
                error_code, obs->scan_control.scan_id);
        }
    }

    if (obs->has_workflow_control)
    {
        uint8_t payload[1] = {obs->workflow_action};
        entry = command_prepare(JETSON_MSG_WORKFLOW_CTRL, obs->workflow_seq,
                                payload, sizeof(payload), &prepare, obs->now_ms);
        if (prepare == COMMAND_NEW)
        {
            command_ack(entry, false, error_code);
        }
    }
    if (obs->has_target_control)
    {
        uint8_t payload[1] = {obs->target_value};
        prepare = COMMAND_NEW;
        entry = command_prepare(JETSON_MSG_TARGET_CTRL, obs->target_seq,
                                payload, sizeof(payload), &prepare, obs->now_ms);
        if (prepare == COMMAND_NEW)
        {
            command_ack(entry, false, error_code);
        }
    }
    if (obs->has_capture_control)
    {
        uint8_t payload[2] = {obs->capture_action, obs->capture_point_id};
        prepare = COMMAND_NEW;
        entry = command_prepare(JETSON_MSG_CAPTURE_CTRL, obs->capture_seq,
                                payload, sizeof(payload), &prepare, obs->now_ms);
        if (prepare == COMMAND_NEW)
        {
            command_ack(entry, false, error_code);
        }
    }
}

void robot_workflow_init(void)
{
    memset(&s_workflow, 0, sizeof(s_workflow));
    memset(s_command_cache, 0, sizeof(s_command_cache));
    s_command_cache_age = 0u;
    memset(&s_scan, 0, sizeof(s_scan));
    memset(&s_stop, 0, sizeof(s_stop));
    s_workflow.state = FLOW_IDLE;
    s_workflow.capture_next_point = 1u;
    robot_capture_init();
    robot_target_init();
    BSP_Laser_Off();
    workflow_update_target_indicators();
}

void robot_workflow_step(const robot_workflow_obs_t *obs)
{
    if (obs == NULL)
    {
        s_workflow.target_aligned = false;
        workflow_force_laser_off(JETSON_ERROR_SAFETY);
        return;
    }

    bool physical_safety_fault = obs->estop_active || obs->limit_triggered ||
                                 workflow_any_limit_triggered();
    if (physical_safety_fault)
    {
        workflow_fault(JETSON_ERROR_SAFETY, obs->now_ms);
        workflow_reject_pending_commands(obs, JETSON_ERROR_SAFETY);
        return;
    }
    if (workflow_is_active() && !obs->heartbeat_alive)
    {
        workflow_fault(JETSON_ERROR_HEARTBEAT_TIMEOUT, obs->now_ms);
        workflow_reject_pending_commands(obs, JETSON_ERROR_HEARTBEAT_TIMEOUT);
        return;
    }
    if (workflow_is_active() && robot_motion_abort_latched() &&
        (s_workflow.state != FLOW_HOMING) &&
        (s_workflow.state != FLOW_RETURN_HOME))
    {
        workflow_fault(JETSON_ERROR_MOTION_ABORTED, obs->now_ms);
        workflow_reject_pending_commands(obs, JETSON_ERROR_MOTION_ABORTED);
        return;
    }

    if (obs->has_workflow_control)
    {
        workflow_handle_workflow_command(obs);
    }
    workflow_stop_step(obs->now_ms);
    if ((s_workflow.state == FLOW_ABORT_HOLD) ||
        (s_workflow.state == FLOW_FAULT_HOLD))
    {
        if (obs->has_target_control)
        {
            (void)workflow_handle_target_command(obs);
        }
        if (obs->has_capture_control)
        {
            workflow_handle_capture_command(obs);
        }
        if (obs->has_scan_control)
        {
            workflow_scan_command(obs);
        }
        workflow_force_laser_off(JETSON_ERROR_NONE);
        return;
    }

    bool target_started = false;
    if (obs->has_target_control)
    {
        target_started = workflow_handle_target_command(obs);
    }

    workflow_handle_safe_distance(obs);
    workflow_handle_reset_and_motion_results(obs->now_ms);
    workflow_handle_retreat_motion(obs->now_ms);

    robot_capture_step(obs->now_ms);
    workflow_handle_capture_result(obs->now_ms);
    workflow_scan_step(obs->now_ms);
    if (obs->has_scan_control)
    {
        workflow_scan_command(obs);
    }
    if (obs->has_capture_control &&
        (s_workflow.state != FLOW_FAULT_HOLD))
    {
        workflow_handle_capture_command(obs);
    }

    if (s_workflow.state == FLOW_TARGET_ACTIVE)
    {
        target_obs_t target_obs =
        {
            .has_vision = obs->has_vision && !target_started,
            .vision_valid = obs->vision_valid,
            .dcx = obs->dcx,
            .dcy = obs->dcy,
            .now_ms = obs->now_ms,
            .fire_button = obs->fire_button,
        };
        robot_target_step(&target_obs);
        workflow_handle_target_event(obs->now_ms);
    }

    workflow_apply_laser_gate(obs);
}

workflow_state_t robot_workflow_state_get(void)
{
    return s_workflow.state;
}

bool robot_workflow_distance_safe_latched(void)
{
    return s_workflow.distance_safe_latched;
}

uint8_t robot_workflow_selected_view_get(void)
{
    return s_workflow.selected_view_id;
}
