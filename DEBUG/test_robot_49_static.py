from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
ROBOT_C = ROOT / "APP" / "robot.c"
DOC = ROOT / "控制文档.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _function_body(source: str, name: str) -> str:
    match = re.search(rf"static\s+\w+\s+{name}\s*\([^)]*\)\s*\{{", source)
    assert match, f"{name} not found"
    start = match.end()
    depth = 1
    idx = start
    while idx < len(source) and depth:
        if source[idx] == "{":
            depth += 1
        elif source[idx] == "}":
            depth -= 1
        idx += 1
    assert depth == 0, f"{name} body not closed"
    return source[start:idx - 1]


def test_joints_sync_waits_for_position_confirm() -> None:
    body = _function_body(_read(ROBOT_C), "robot_joints_sync_to")
    assert "robot_joint_wait_target" in body
    assert "robot_joint_stop" in body


def test_control_doc_has_no_stale_settle_period_description() -> None:
    doc = _read(DOC)
    assert "ROBOT_PID_SETTLE_PERIODS`（10，通用终点稳定）均保留不变" not in doc
    assert "ROBOT_PID_SETTLE_PERIODS`（150，通用终点自适应稳定上限）" in doc


def test_control_doc_names_time_func_move_for_pid_return_fix() -> None:
    doc = _read(DOC)
    section = doc.split("#### 4.9.5", 1)[1].split("### 4.10", 1)[0]
    assert "robot_time_func_move()" in section
    assert "robot_time_func_path_interpolation()` 原来的判断" not in section


if __name__ == "__main__":
    test_joints_sync_waits_for_position_confirm()
    test_control_doc_has_no_stale_settle_period_description()
    test_control_doc_names_time_func_move_for_pid_return_fix()
    print("robot 4.9 static checks passed")
