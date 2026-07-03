from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ROBOT_C = (ROOT / "APP" / "robot.c").read_text(encoding="utf-8")
DOC = (ROOT / "控制文档.md").read_text(encoding="utf-8")


def require(text: str, needle: str, label: str) -> None:
    assert needle in text, f"missing {label}: {needle}"


def test_joint_position_wait_helper_exists() -> None:
    require(ROBOT_C, "static bool robot_joint_wait_target", "joint wait helper")
    require(ROBOT_C, "robot_joint_compare_error", "shared error compare")
    require(ROBOT_C, "ROBOT_JOINT_POS_CONFIRM_STABLE_HITS", "stable hit threshold")


def test_joint_position_commands_wait_after_send() -> None:
    rel_case = ROBOT_C.split("case ROBOT_JOINT_REL_ROTATE:", 1)[1].split("break;", 1)[0]
    abs_case = ROBOT_C.split("case ROBOT_JOINT_ABS_ROTATE:", 1)[1].split("break;", 1)[0]
    require(rel_case, "robot_joint_wait_target", "relative joint command wait")
    require(abs_case, "robot_joint_wait_target", "absolute joint command wait")


def test_auto_updates_pose_only_after_final_confirm() -> None:
    auto_func = ROBOT_C.split("static void robot_auto_move_interpolation", 1)[1].split("static float robot_angle_normalize", 1)[0]
    require(auto_func, "robot_auto_final_confirm", "AUTO final confirm")
    assert "g_robot.cur_pos.x = target_pos->x;" in auto_func
    assert auto_func.index("robot_auto_final_confirm") < auto_func.index("g_robot.cur_pos.x = target_pos->x;")
    require(auto_func, "ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_POSE_VALID)", "pose invalid on AUTO miss")


def test_time_func_updates_pose_only_after_final_confirm() -> None:
    time_func = ROBOT_C.split("static void robot_time_func_move", 1)[1].split("static void robot_auto_busy_set", 1)[0]
    require(time_func, "robot_auto_final_confirm", "time_func final confirm")
    assert "g_robot.cur_pos.x = path[path_size -1].x;" in time_func
    assert time_func.index("robot_auto_final_confirm") < time_func.index("g_robot.cur_pos.x = path[path_size -1].x;")


def test_document_marks_49_done() -> None:
    require(DOC, "| 4.9  | 位置模式到位确认       | 已完成", "4.9 completed status")
    require(DOC, "robot_joint_wait_target", "document joint wait helper")
    require(DOC, "robot_auto_final_confirm", "document auto final confirm")


if __name__ == "__main__":
    tests = [
        test_joint_position_wait_helper_exists,
        test_joint_position_commands_wait_after_send,
        test_auto_updates_pose_only_after_final_confirm,
        test_time_func_updates_pose_only_after_final_confirm,
        test_document_marks_49_done,
    ]
    for test in tests:
        test()
    print("4.9 static checks passed")
