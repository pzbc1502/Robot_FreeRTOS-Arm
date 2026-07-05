from pathlib import Path
import sys
import tempfile

from PySide6.QtCore import QSettings
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


def _clear_settings() -> None:
    settings = QSettings("RA6M5Robot", "UpperConsole")
    settings.clear()
    settings.sync()


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


def test_target_demo_controls_are_visible_with_safe_defaults() -> None:
    _clear_settings()
    window = _new_window()
    try:
        texts = _button_texts(window)
        assert "一键定靶演示" in texts
        assert "停止演示" in texts
        assert "保存演示配置" in texts
        assert window.demo_sequence.text() == "30,-20:5;15,-10:5;8,-5:5;0,0:0"
        assert window.demo_period_ms.text() == "200"
        assert window.demo_send_enable.isChecked()
        assert window.demo_wait_ready.isChecked()
        assert window.demo_auto_stop.isChecked()
    finally:
        window.close()
        _clear_settings()


def test_target_demo_settings_persist() -> None:
    _clear_settings()
    window = _new_window()
    try:
        window.demo_sequence.setText("15,-10:2;0,0:0")
        window.demo_period_ms.setText("150")
        window.demo_wait_ready.setChecked(False)
        window.save_demo_settings()
    finally:
        window.close()

    window = _new_window()
    try:
        assert window.demo_sequence.text() == "15,-10:2;0,0:0"
        assert window.demo_period_ms.text() == "150"
        assert not window.demo_wait_ready.isChecked()
    finally:
        window.close()
        _clear_settings()


def test_status_lights_follow_ra6_status() -> None:
    window = _new_window()
    try:
        window._update_ra6_status("TARGET_CTRL_ON")
        assert "background-color: #22c55e" in window.status_dots["TARGET_CTRL"].styleSheet()
        window._update_ra6_status("OUTPUT_OFF")
        assert "background-color: #9ca3af" in window.status_dots["OUTPUT"].styleSheet()
        window._update_ra6_status("ERROR")
        assert "background-color: #ef4444" in window.status_dots["ERROR"].styleSheet()
    finally:
        window.close()


def test_closing_ports_clears_unknown_runtime_status_lights() -> None:
    window = _new_window()
    try:
        window._set_status("POSE_VALID", "YES")
        window.close_channel(window.arm)
        assert window.status_labels["POSE_VALID"].text() == "POSE_VALID: -"
        assert "background-color: #9ca3af" in window.status_dots["POSE_VALID"].styleSheet()

        for name in ("TARGET_CTRL_ON", "READY", "ALIGN_DONE", "OUTPUT_ON", "ERROR"):
            window._update_ra6_status(name)
        window.close_channel(window.jetson)
        for key in ("TARGET_CTRL", "READY", "ALIGN_DONE", "CONFIRMING", "OUTPUT", "ERROR"):
            assert window.status_labels[key].text() == f"{key}: -"
            assert "background-color: #9ca3af" in window.status_dots[key].styleSheet()
    finally:
        window.close()


def test_target_restart_and_stop_clear_stale_target_lights() -> None:
    window = _new_window()
    try:
        for name in ("TARGET_CTRL_ON", "READY", "ALIGN_DONE", "OUTPUT_ON", "ERROR"):
            window._update_ra6_status(name)

        window._update_ra6_status("TARGET_CTRL_ON")
        for key in ("READY", "ALIGN_DONE", "CONFIRMING", "OUTPUT", "ERROR"):
            assert window.status_labels[key].text() == f"{key}: -"
            assert "background-color: #9ca3af" in window.status_dots[key].styleSheet()

        window._update_ra6_status("READY")
        window._update_ra6_status("TARGET_CTRL_OFF")
        assert window.status_labels["TARGET_CTRL"].text() == "TARGET_CTRL: OFF"
        assert window.status_labels["OUTPUT"].text() == "OUTPUT: OFF"
        for key in ("READY", "ALIGN_DONE", "CONFIRMING", "ERROR"):
            assert window.status_labels[key].text() == f"{key}: -"
            assert "background-color: #9ca3af" in window.status_dots[key].styleSheet()
    finally:
        window.close()


if __name__ == "__main__":
    test_log_clear_and_refresh_controls_are_visible()
    test_arm_command_without_open_port_has_chinese_log_feedback()
    test_soft_reset_button_sends_soft_reset_command()
    test_clear_log_clears_visible_log_area()
    test_arm_command_history_keeps_latest_15_and_moves_duplicates_to_front()
    test_target_demo_controls_are_visible_with_safe_defaults()
    test_target_demo_settings_persist()
    test_status_lights_follow_ra6_status()
    test_closing_ports_clears_unknown_runtime_status_lights()
    test_target_restart_and_stop_clear_stale_target_lights()
    print("upper console gui feature checks passed")
