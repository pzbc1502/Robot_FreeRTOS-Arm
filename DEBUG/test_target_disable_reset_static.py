from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ROBOT_TARGET_C = (ROOT / "APP" / "robot_target.c").read_text(encoding="utf-8")


def _function_body(name: str) -> str:
    start = ROBOT_TARGET_C.index(f"void {name}(void)")
    next_func = ROBOT_TARGET_C.index("\nvoid ", start + 1)
    return ROBOT_TARGET_C[start:next_func]


def test_target_disable_requests_soft_reset() -> None:
    body = _function_body("robot_target_disable_request")
    assert "ROBOT_TARGET_ENABLED = false" in body
    assert "force_laser_off()" in body
    assert "robot_send_reset_event(false)" in body


if __name__ == "__main__":
    test_target_disable_requests_soft_reset()
    print("target disable reset static check passed")
