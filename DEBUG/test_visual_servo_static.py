from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ROBOT_H = (ROOT / "APP" / "robot.h").read_text(encoding="utf-8")
ROBOT_C = (ROOT / "APP" / "robot.c").read_text(encoding="utf-8")
TARGET_H = (ROOT / "APP" / "robot_target.h").read_text(encoding="utf-8")
TARGET_C = (ROOT / "APP" / "robot_target.c").read_text(encoding="utf-8")


def require(text: str, needle: str, label: str) -> None:
    assert needle in text, f"missing {label}: {needle}"


def case_body(text: str, case_label: str, next_case_label: str) -> str:
    start = text.rindex(case_label)
    end = text.index(next_case_label, start)
    return text[start:end]


def test_robot_visual_servo_api_exists() -> None:
    require(ROBOT_H, "ROBOT_VISUAL_SERVO_EVENT", "visual servo event")
    require(ROBOT_H, "ROBOT_STATUS_VISUAL_SERVO_ACTIVE", "visual servo status")
    require(ROBOT_H, "int robot_visual_servo_start(void);", "visual servo start API")
    require(ROBOT_H, "void robot_visual_servo_stop(void);", "visual servo stop API")
    require(ROBOT_H, "void robot_visual_servo_set_velocity(float vx, float vy, float vz);", "visual servo velocity API")
    require(ROBOT_H, "bool robot_is_visual_servo_active(void);", "visual servo active API")
    require(ROBOT_H, "void robot_motion_abort(void);", "motion abort API")
    require(ROBOT_H, "robot_auto_result_t", "auto result type")
    require(ROBOT_H, "robot_auto_result_t robot_auto_result_consume(void);", "auto result consume API")


def test_robot_visual_servo_runtime_exists() -> None:
    require(ROBOT_C, "static int robot_visual_servo_run(void)", "visual servo runner")
    require(ROBOT_C, "case ROBOT_VISUAL_SERVO_EVENT:", "visual servo event dispatch")
    require(ROBOT_C, "ROBOT_STATUS_SET(g_robot.status, ROBOT_STATUS_VISUAL_SERVO_ACTIVE)", "visual servo active set")
    require(ROBOT_C, "ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_VISUAL_SERVO_ACTIVE)", "visual servo active clear")
    require(ROBOT_C, "ROBOT_STATUS_SET(g_robot.status, ROBOT_STATUS_AUTO_BUSY)", "auto busy guard")
    require(ROBOT_C, "robot_pid_one_period(target_angle, feedforward, NULL, ROBOT_ARM_JOINT_NUM)", "shared PID use")
    require(ROBOT_C, "robot_motion_abort_is_requested()", "motion abort check")
    require(ROBOT_C, "robot pid run aborted by safety request", "motion abort log")
    require(ROBOT_C, "robot_auto_result_set(ROBOT_AUTO_RESULT_RUNNING)", "auto result running set")
    require(ROBOT_C, "robot_auto_result_set(ROBOT_AUTO_RESULT_OK)", "auto result ok set")
    require(ROBOT_C, "robot_auto_result_set(ROBOT_AUTO_RESULT_FAILED)", "auto result failed set")
    require(ROBOT_C, "robot_auto_result_set(ROBOT_AUTO_RESULT_ABORTED)", "auto result aborted set")


def test_target_visual_servo_params_and_calls_exist() -> None:
    require(TARGET_H, "TARGET_USE_VISUAL_SERVO", "visual servo compile switch")
    require(TARGET_H, "TARGET_VS_KX_MM_S_PER_PX", "visual servo X gain")
    require(TARGET_H, "TARGET_VS_KZ_MM_S_PER_PX", "visual servo Z gain")
    require(TARGET_H, "TARGET_VS_MAX_SPEED_MM_S", "visual servo coarse speed")
    require(TARGET_H, "TARGET_VS_FINE_MAX_SPEED_MM_S", "visual servo fine speed")
    require(TARGET_H, "TARGET_VS_CMD_TIMEOUT_MS", "visual servo command timeout")
    require(TARGET_H, "TARGET_SAFE_DISTANCE_MM", "safe distance threshold")
    require(TARGET_H, "TARGET_SAFE_RETREAT_SPEED_MM_S", "safe distance retreat speed")
    require(TARGET_H, "TARGET_READY_STATUS_PERIOD_MS", "READY status retry period")
    require(TARGET_C, "#if TARGET_USE_VISUAL_SERVO", "visual servo target branch")
    require(TARGET_C, "robot_visual_servo_start()", "target starts visual servo")
    require(TARGET_C, "robot_visual_servo_set_velocity", "target updates visual servo velocity")
    require(TARGET_C, "robot_visual_servo_stop()", "target stops visual servo")
    require(TARGET_C, "target_handle_safe_distance_guard", "safe distance guard")
    require(TARGET_C, "if (!safe_distance_fresh(now))", "pre-position waits for safe distance")
    require(TARGET_C, "robot_motion_abort();", "safe distance aborts current motion")


def test_pre_position_ready_uses_coarse_auto_completion() -> None:
    pre_position = case_body(TARGET_C, "case TARGET_PRE_POSITION:", "case TARGET_WAIT_DETECT:")
    assert "position_near(" not in pre_position, "READY must not require exact Cartesian target after coarse AUTO"
    assert "robot_is_auto_busy()" not in pre_position, "pre-position should consume explicit AUTO result, not poll busy"
    require(pre_position, "robot_auto_result_t auto_result = robot_auto_result_consume();", "auto result consume")
    require(pre_position, "case ROBOT_AUTO_RESULT_OK:", "auto ok branch")
    require(pre_position, "case ROBOT_AUTO_RESULT_FAILED:", "auto failed branch")
    require(pre_position, "case ROBOT_AUTO_RESULT_ABORTED:", "auto aborted branch")
    require(pre_position, "jetson_send_status_u8(RA6_TO_JETSON_READY, 1u)", "READY status after coarse auto")
    require(pre_position, "s_target.last_ready_status_ms = now;", "READY status timestamp after coarse auto")


def test_wait_detect_repeats_ready_status_until_vision_arrives() -> None:
    wait_detect = case_body(TARGET_C, "case TARGET_WAIT_DETECT:", "case TARGET_ALIGN:")
    require(TARGET_C, "uint32_t last_ready_status_ms;", "READY resend timestamp")
    require(wait_detect, "TARGET_READY_STATUS_PERIOD_MS", "READY resend period in wait detect")
    require(wait_detect, "jetson_send_status_u8(RA6_TO_JETSON_READY, 1u)", "READY resend in wait detect")
    require(wait_detect, "s_target.last_ready_status_ms = now;", "READY resend timestamp update")


if __name__ == "__main__":
    test_robot_visual_servo_api_exists()
    test_robot_visual_servo_runtime_exists()
    test_target_visual_servo_params_and_calls_exist()
    test_pre_position_ready_uses_coarse_auto_completion()
    test_wait_detect_repeats_ready_status_until_vision_arrives()
    print("visual servo static checks passed")
