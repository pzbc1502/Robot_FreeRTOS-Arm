from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "UpperComputer" / "ra6m5_upper_console.py").read_text(encoding="utf-8")


def test_capture_buttons_are_chinese() -> None:
    required = [
        "左视图拍摄",
        "正视图拍摄",
        "右视图拍摄",
        "三视图完成回HOME",
        "选择左视图定靶点",
        "选择正视图定靶点",
        "选择右视图定靶点",
    ]
    for label in required:
        assert label in SOURCE


def test_capture_buttons_do_not_use_old_english_labels() -> None:
    old_labels = [
        "Capture L",
        "Capture F",
        "Capture R",
        "Capture Done HOME",
        "Select L prestart",
        "Select F prestart",
        "Select R prestart",
    ]
    for label in old_labels:
        assert label not in SOURCE


def test_fire_key_prompts_use_p002_switch() -> None:
    assert "视觉对准完成，请先断开再接通 P002 开关" in SOURCE
    assert "激光已开启，断开 P002 开关可关闭" in SOURCE
    assert 'QCheckBox("对准后提示拨动P002")' in SOURCE
    assert "请按住 P000" not in SOURCE
    assert "请松开 P000" not in SOURCE
    assert "demo_prompt_p000" in SOURCE  # Keep the existing settings field compatible.


if __name__ == "__main__":
    test_capture_buttons_are_chinese()
    test_capture_buttons_do_not_use_old_english_labels()
    test_fire_key_prompts_use_p002_switch()
    print("upper console Chinese label checks passed")
