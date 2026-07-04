from pathlib import Path
import sys
import tempfile

from PySide6.QtWidgets import QApplication, QPushButton

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from UpperComputer.ra6m5_upper_console import QtUpperConsole  # noqa: E402


def _new_window() -> QtUpperConsole:
    QApplication.instance() or QApplication([])
    temp_dir = tempfile.TemporaryDirectory()
    window = QtUpperConsole(Path(temp_dir.name))
    window._test_temp_dir = temp_dir
    return window


def _button_texts(window: QtUpperConsole) -> list[str]:
    return [button.text() for button in window.findChildren(QPushButton)]


def _button_by_text(window: QtUpperConsole, text: str) -> QPushButton:
    for button in window.findChildren(QPushButton):
        if button.text() == text:
            return button
    raise AssertionError(f"button not found: {text}")


def test_log_clear_and_refresh_controls_are_visible() -> None:
    window = _new_window()
    try:
        texts = _button_texts(window)
        assert "清空日志" in texts
        assert "刷新串口" in texts
    finally:
        window.close()


def test_arm_command_without_open_port_has_chinese_log_feedback() -> None:
    window = _new_window()
    try:
        window.send_arm_command("soft_reset")
        window._flush_logs()
        text = window.log_text.toPlainText()
        assert "ARM串口未打开" in text
        assert "soft_reset" in text
    finally:
        window.close()


def test_soft_reset_button_sends_soft_reset_command() -> None:
    window = _new_window()
    try:
        _button_by_text(window, "soft_reset").click()
        window._flush_logs()
        text = window.log_text.toPlainText()
        assert "ARM串口未打开" in text
        assert "soft_reset" in text
        assert "False" not in text
    finally:
        window.close()


def test_clear_log_clears_visible_log_area() -> None:
    window = _new_window()
    try:
        window.emit_log("APP", "INFO", "hello")
        window._flush_logs()
        assert "hello" in window.log_text.toPlainText()
        window.clear_visible_log()
        assert window.log_text.toPlainText() == ""
    finally:
        window.close()


def test_arm_command_history_keeps_latest_15_and_moves_duplicates_to_front() -> None:
    window = _new_window()
    try:
        for idx in range(17):
            window.send_arm_command(f"cmd_{idx}")
        history = [window.arm_history.itemText(idx) for idx in range(window.arm_history.count())]
        assert len(history) == 15
        assert history[0] == "cmd_16"
        assert history[-1] == "cmd_2"

        window.send_arm_command("cmd_10")
        history = [window.arm_history.itemText(idx) for idx in range(window.arm_history.count())]
        assert history[0] == "cmd_10"
        assert history.count("cmd_10") == 1
        assert len(history) == 15
    finally:
        window.close()


if __name__ == "__main__":
    test_log_clear_and_refresh_controls_are_visible()
    test_arm_command_without_open_port_has_chinese_log_feedback()
    test_soft_reset_button_sends_soft_reset_command()
    test_clear_log_clears_visible_log_area()
    test_arm_command_history_keeps_latest_15_and_moves_duplicates_to_front()
    print("upper console gui feature checks passed")
