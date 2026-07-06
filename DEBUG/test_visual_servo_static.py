from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ROBOT_H = (ROOT / "APP" / "robot.h").read_text(encoding="utf-8")
ROBOT_C = (ROOT / "APP" / "robot.c").read_text(encoding="utf-8")
TARGET_H = (ROOT / "APP" / "robot_target.h").read_text(encoding="utf-8")
TARGET_C = (ROOT / "APP" / "robot_target.c").read_text(encoding="utf-8")


def require(text: str, needle: str, label: str) -> None:
    assert needle in text, f"missing {label}: {needle}"


def test_robot_visual_servo_api_exists() -> None:
    require(ROBOT_H, "ROBOT_VISUAL_SERVO_EVENT", "visual servo event")
    require(ROBOT_H, "ROBOT_STATUS_VISUAL_SERVO_ACTIVE", "visual servo status")
    require(ROBOT_H, "int robot_visual_servo_start(void);", "visual servo start API")
    require(ROBOT_H, "void robot_visual_servo_stop(void);", "visual servo stop API")
    require(ROBOT_H, "void robot_visual_servo_set_velocity(float vx, float vy, float vz);", "visual servo velocity API")
    require(ROBOT_H, "bool robot_is_visual_servo_active(void);", "visual servo active API")


def test_robot_visual_servo_runtime_exists() -> None:
    require(ROBOT_C, "static int robot_visual_servo_run(void)", "visual servo runner")
    require(ROBOT_C, "case ROBOT_VISUAL_SERVO_EVENT:", "visual servo event dispatch")
    require(ROBOT_C, "ROBOT_STATUS_SET(g_robot.status, ROBOT_STATUS_VISUAL_SERVO_ACTIVE)", "visual servo active set")
    require(ROBOT_C, "ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_VISUAL_SERVO_ACTIVE)", "visual servo active clear")
    require(ROBOT_C, "ROBOT_STATUS_SET(g_robot.status, ROBOT_STATUS_AUTO_BUSY)", "auto busy guard")
    require(ROBOT_C, "robot_pid_one_period(target_angle, feedforward, NULL, ROBOT_ARM_JOINT_NUM)", "shared PID use")


def test_target_visual_servo_params_and_calls_exist() -> None:
    require(TARGET_H, "TARGET_USE_VISUAL_SERVO", "visual servo compile switch")
    require(TARGET_H, "TARGET_VS_KX_MM_S_PER_PX", "visual servo X gain")
    require(TARGET_H, "TARGET_VS_KZ_MM_S_PER_PX", "visual servo Z gain")
    require(TARGET_H, "TARGET_VS_MAX_SPEED_MM_S", "visual servo coarse speed")
    require(TARGET_H, "TARGET_VS_FINE_MAX_SPEED_MM_S", "visual servo fine speed")
    require(TARGET_H, "TARGET_VS_CMD_TIMEOUT_MS", "visual servo command timeout")
    require(TARGET_H, "TARGET_SAFE_DISTANCE_MM", "safe distance threshold")
    require(TARGET_H, "TARGET_SAFE_RETREAT_SPEED_MM_S", "safe distance retreat speed")
    require(TARGET_C, "#if TARGET_USE_VISUAL_SERVO", "visual servo target branch")
    require(TARGET_C, "robot_visual_servo_start()", "target starts visual servo")
    require(TARGET_C, "robot_visual_servo_set_velocity", "target updates visual servo velocity")
    require(TARGET_C, "robot_visual_servo_stop()", "target stops visual servo")
    require(TARGET_C, "target_handle_safe_distance_guard", "safe distance guard")


if __name__ == "__main__":
    test_robot_visual_servo_api_exists()
    test_robot_visual_servo_runtime_exists()
    test_target_visual_servo_params_and_calls_exist()
    print("visual servo static checks passed")
