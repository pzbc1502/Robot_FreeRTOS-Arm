"""Test the Qt log display filter without opening serial ports or user settings."""
import os
os.environ["QT_QPA_PLATFORM"] = "offscreen"

from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from PySide6.QtWidgets import QApplication
from UpperComputer import ra6m5_upper_console as ui

UART_STATS = (
    "[JETSON_UART] rx_bytes=0 frames_ok=0 crc_fail=0 overflow=0 "
    "tx_ok=0 tx_start_fail=0 tx_timeout=0 last_type=0x00 last_seq=0"
)


class Settings:
    def __init__(self):
        self.data = {}

    def value(self, key, default=None):
        return self.data.get(key, default)

    def setValue(self, key, value):
        self.data[key] = value

    def sync(self):
        pass


class LogFilterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="console_log_filter_")
        self.addCleanup(self.directory.cleanup)
        self.settings = Settings()
        self.settings_patch = patch.object(ui, "QSettings", return_value=self.settings)
        self.settings_patch.start()
        self.addCleanup(self.settings_patch.stop)
        self.ports_patch = patch.object(ui.list_ports, "comports", return_value=[])
        self.ports_patch.start()
        self.addCleanup(self.ports_patch.stop)
        self.window = self.new_window()
        self.addCleanup(self.window.close)

    def new_window(self):
        window = ui.QtUpperConsole(Path(self.directory.name))
        window.timer.stop()
        window._flush_logs()
        window.clear_visible_log()
        return window

    def test_default_hidden_but_written_to_disk(self):
        self.assertFalse(self.window.show_uart_stats.isChecked())
        self.window.on_serial_data("ARM", (UART_STATS + "\r\n").encode())
        self.window._flush_logs()
        self.assertNotIn(UART_STATS, self.window.log_text.toPlainText())
        self.assertIn(UART_STATS, self.window.logger.path.read_text(encoding="utf-8"))
        self.assertEqual(len(self.window.log_history), 1)

    def test_toggle_rebuilds_history_and_future_display(self):
        self.window.emit_log("ARM", "RX", UART_STATS)
        self.window.emit_log("ARM", "RX", "[TARGET] vision lost")
        self.window._flush_logs()
        self.window.show_uart_stats.setChecked(True)
        self.assertIn(UART_STATS, self.window.log_text.toPlainText())
        self.assertIn("vision lost", self.window.log_text.toPlainText())
        self.window.emit_log("ARM", "RX", UART_STATS.replace("rx_bytes=0", "rx_bytes=12"))
        self.window._flush_logs()
        self.assertIn("rx_bytes=12", self.window.log_text.toPlainText())
        self.window.show_uart_stats.setChecked(False)
        self.assertNotIn("[JETSON_UART] rx_bytes=", self.window.log_text.toPlainText())
        self.assertIn("vision lost", self.window.log_text.toPlainText())

    def test_unrelated_uart_and_safety_messages_stay_visible(self):
        messages = ["[JETSON_UART] TX timeout", "motion_abort latched", "[AUTO] failed"]
        for message in messages:
            self.window.emit_log("ARM", "RX", message)
        self.window.emit_log("ARM", "ERROR", UART_STATS)
        self.window._flush_logs()
        for message in messages:
            self.assertIn(message, self.window.log_text.toPlainText())
        self.assertIn("[ARM-ERROR] " + UART_STATS, self.window.log_text.toPlainText())

    def test_clear_does_not_restore_old_lines_or_truncate_file(self):
        self.window.emit_log("ARM", "RX", UART_STATS)
        self.window._flush_logs()
        self.window.clear_visible_log()
        self.window.show_uart_stats.setChecked(True)
        self.assertEqual(self.window.log_text.toPlainText(), "")
        self.assertEqual(len(self.window.log_history), 0)
        self.assertIn(UART_STATS, self.window.logger.path.read_text(encoding="utf-8"))

    def test_setting_survives_window_recreation(self):
        self.window.show_uart_stats.setChecked(True)
        self.assertTrue(self.settings.value("logs/show_uart_stats"))
        other = self.new_window()
        try:
            self.assertTrue(other.show_uart_stats.isChecked())
        finally:
            other.close()

    def test_status_parsing_remains_active_when_hidden(self):
        self.window.on_serial_data(
            "ARM", (UART_STATS + "\r\nsoft reset final verify pass\r\n").encode()
        )
        self.window._flush_logs()
        self.assertTrue(self.window.arm_events["SOFT_RESET_PASS"].is_set())
        self.assertEqual(self.window.status_labels["POSE_VALID"].text(), "POSE_VALID: YES")
        self.assertNotIn(UART_STATS, self.window.log_text.toPlainText())

    def test_history_is_bounded(self):
        with patch.object(ui, "LOG_HISTORY_LIMIT", 3):
            other = self.new_window()
        try:
            for index in range(5):
                other.emit_log("APP", "INFO", f"bounded_line_{index}")
            other._flush_logs()
            self.assertEqual(len(other.log_history), 3)
            other.show_uart_stats.setChecked(True)
            text = other.log_text.toPlainText()
            self.assertNotIn("bounded_line_1", text)
            self.assertIn("bounded_line_2", text)
            self.assertIn("bounded_line_4", text)
        finally:
            other.close()


if __name__ == "__main__":
    unittest.main()
