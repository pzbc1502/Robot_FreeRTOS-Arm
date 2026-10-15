#ifndef ROBOT_WORKFLOW_H_
#define ROBOT_WORKFLOW_H_

#include <stdint.h>
#include <stdbool.h>
#include "jetson_vision.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOT_WORKFLOW_MEASURE_X              (0.0f)
#define ROBOT_WORKFLOW_MEASURE_Y              (-130.0f)
#define ROBOT_WORKFLOW_MEASURE_Z              (-15.0f)
#define ROBOT_WORKFLOW_SAFE_DISTANCE_MM       (100u)
#define ROBOT_WORKFLOW_SAFE_STABLE_COUNT      (3u)
#define ROBOT_WORKFLOW_RETREAT_STEP_MM        (20.0f)
#define ROBOT_WORKFLOW_RETREAT_MAX_STEPS      (8u)
#define ROBOT_WORKFLOW_RETREAT_WAIT_MS        (2000u)
#define ROBOT_WORKFLOW_MOTION_TIMEOUT_MS      (15000u)
#define ROBOT_WORKFLOW_RESET_TIMEOUT_MS       (45000u)
#define ROBOT_WORKFLOW_GLOBAL_RESET_REPLAY_MS (60000u)
#define ROBOT_WORKFLOW_RESET_KEY_HOLD_MS      (2000u)
#define ROBOT_WORKFLOW_RESET_KEY_RELEASE_MS   (40u)
#define ROBOT_WORKFLOW_RESET_KEY_SAMPLE_GAP_MS (250u)
/* Stop confirmation observes feedback stability, not a hardware emergency stop. */
#define ROBOT_WORKFLOW_STOP_TIMEOUT_MS        (2000u)
#define ROBOT_WORKFLOW_STOP_SAMPLE_MS         (100u)
#define ROBOT_WORKFLOW_STOP_STABLE_MS         (200u)
#define ROBOT_WORKFLOW_STOP_STABLE_COUNT      (3u)
#define ROBOT_WORKFLOW_STOP_TOL_DEG           (0.10f)
#define ROBOT_SCAN_PREPARE_TIMEOUT_MS          (60000u)
#define ROBOT_SCAN_SEGMENT_TIMEOUT_MS          (30000u)

typedef enum
{
    FLOW_IDLE = 0,
    FLOW_HOMING,
    FLOW_MEASURE_POSITION,
    FLOW_WAIT_SAFE_DISTANCE,
    FLOW_SAFE_READY,
    FLOW_CAPTURE,
    FLOW_CAPTURE_HOME,
    FLOW_SELECTED_VIEW,
    FLOW_TARGET_ACTIVE,
    FLOW_TARGET_HOLD,
    FLOW_RETURN_HOME,
    FLOW_RETREAT_WAIT_RESTART,
    FLOW_ABORT_HOLD,
    FLOW_FAULT_HOLD,
    FLOW_SCAN_ACTIVE,
    FLOW_SCAN_READY,
} workflow_state_t;

typedef struct
{
    uint32_t now_ms;
    bool heartbeat_alive;
    bool estop_active;
    bool limit_triggered;
    bool fire_button;
    bool reset_button;
    bool reset_button_valid;

    bool has_workflow_control;
    uint8_t workflow_action;
    uint8_t workflow_seq;

    bool has_scan_control;
    jetson_scan_control_t scan_control;

    bool has_capture_control;
    uint8_t capture_action;
    uint8_t capture_point_id;
    uint8_t capture_seq;

    bool has_target_control;
    uint8_t target_value;
    uint8_t target_seq;

    bool has_vision;
    bool vision_valid;
    int16_t dcx;
    int16_t dcy;
    uint8_t vision_seq;
    uint32_t vision_received_ms;

    bool has_distance;
    bool distance_valid;
    uint16_t distance_mm;
    uint8_t distance_seq;
} robot_workflow_obs_t;

void robot_workflow_init(void);
void robot_workflow_step(const robot_workflow_obs_t *obs);
workflow_state_t robot_workflow_state_get(void);
bool robot_workflow_distance_safe_latched(void);
uint8_t robot_workflow_selected_view_get(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_WORKFLOW_H_ */
