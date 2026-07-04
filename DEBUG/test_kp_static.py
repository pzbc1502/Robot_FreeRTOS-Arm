from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
ROBOT_C = (ROOT / "APP" / "robot.c").read_text(encoding="utf-8")
DOC = (ROOT / "控制文档.md").read_text(encoding="utf-8")

EXPECTED_KP = "{0.65f, 4.0f, 2.50f, 3.50f, 4.50f, 10.0f}"
EXPECTED_DOC_KP = "{0.65, 4.00, 2.50, 3.50, 4.50, 10.0}"


def test_robot_joint_kp_matches_tuned_values() -> None:
    match = re.search(r"ROBOT_JOINT_KP\[ROBOT_MAX_JOINT_NUM\]\s*=\s*(\{[^;]+\})", ROBOT_C)
    assert match, "ROBOT_JOINT_KP definition not found"
    assert match.group(1) == EXPECTED_KP


def test_control_doc_records_current_kp_values() -> None:
    assert EXPECTED_DOC_KP in DOC


if __name__ == "__main__":
    test_robot_joint_kp_matches_tuned_values()
    test_control_doc_records_current_kp_values()
    print("Kp static checks passed")
