from __future__ import annotations

import argparse
import datetime as dt
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import serial
from serial.tools import list_ports
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

APP_TITLE = "RA6M5 Robot Serial Console"
DEFAULT_BAUD = 115200
ARM_HISTORY_LIMIT = 15
JETSON_SOF = 0xFF
JETSON_EOF = 0xFE
JETSON_LEN = 0x05
JETSON_FUNC_VISION_ERROR = 0x03
JETSON_CTRL_SOF = 0xAA
JETSON_CTRL_EOF = 0xBB
JETSON_FUNC_TARGET_CTRL = 0x01
RA6_SOF = 0xCC
RA6_EOF = 0xDD
STATUS_NAMES = {
    (0x01, 0x01): "READY",
    (0x02, 0x01): "ALIGN_DONE",
    (0x03, 0x01): "OUTPUT_ON",
    (0x03, 0x00): "OUTPUT_OFF",
    (0x04, 0x01): "TARGET_CTRL_ON",
    (0x04, 0x00): "TARGET_CTRL_OFF",
    (0xFE, 0x01): "ERROR",
}


@dataclass(frozen=True)
class Ra6Status:
    raw: bytes
    func: int
    value: int
    name: str


def bytes_to_hex(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def build_arm_command(command: str) -> bytes:
    return (command.strip() + "\r\n").encode("ascii", errors="replace")


def _int16_le(value: int) -> bytes:
    if value < -32768 or value > 32767:
        raise ValueError("vision error must fit int16")
    return int(value).to_bytes(2, "little", signed=True)


def build_vision_error_frame(dcx: int, dcy: int) -> bytes:
    payload = _int16_le(dcx) + _int16_le(dcy)
    checksum = (JETSON_LEN + JETSON_FUNC_VISION_ERROR + sum(payload)) & 0xFF
    return bytes([JETSON_SOF, JETSON_LEN, JETSON_FUNC_VISION_ERROR]) + payload + bytes([checksum, JETSON_EOF])


def build_target_control_frame(enable: bool) -> bytes:
    return bytes([JETSON_CTRL_SOF, JETSON_FUNC_TARGET_CTRL, 0x01 if enable else 0x00, JETSON_CTRL_EOF])


def parse_ra6_status_frames(data: bytes) -> list[Ra6Status]:
    frames: list[Ra6Status] = []
    idx = 0
    while idx + 3 < len(data):
        if data[idx] != RA6_SOF:
            idx += 1
            continue
        raw = data[idx:idx + 4]
        if len(raw) == 4 and raw[3] == RA6_EOF:
            func = raw[1]
            value = raw[2]
            frames.append(Ra6Status(raw=raw, func=func, value=value, name=STATUS_NAMES.get((func, value), "UNKNOWN")))
            idx += 4
        else:
            idx += 1
    return frames


def parse_vision_sequence(text: str) -> list[tuple[int, int, int]]:
    items: list[tuple[int, int, int]] = []
    for part in text.split(";"):
        part = part.strip()
        if not part:
            continue
        xy_text, _, count_text = part.partition(":")
        x_text, comma, y_text = xy_text.partition(",")
        if not comma:
            raise ValueError(f"bad vision sequence item: {part}")
        count = int(count_text) if count_text else 1
        if count < 0:
            raise ValueError("vision sequence count must be >= 0")
        items.append((int(x_text.strip()), int(y_text.strip()), count))
    return items


def protocol_self_test() -> None:
    assert build_vision_error_frame(-7, -50) == bytes.fromhex("FF 05 03 F9 FF CE FF CD FE")
    assert build_target_control_frame(True) == bytes.fromhex("AA 01 01 BB")
    assert build_target_control_frame(False) == bytes.fromhex("AA 01 00 BB")
    assert build_arm_command(" soft_reset ") == b"soft_reset\r\n"
    frames = parse_ra6_status_frames(bytes.fromhex("00 CC 04 01 DD 99 CC 03 00 DD"))
    assert [(item.func, item.value, item.name) for item in frames] == [
        (0x04, 0x01, "TARGET_CTRL_ON"),
        (0x03, 0x00, "OUTPUT_OFF"),
    ]
    assert parse_vision_sequence("15,-10:2;8,-5:1;0,0:0") == [(15, -10, 2), (8, -5, 1), (0, 0, 0)]


class SessionLogger:
    def __init__(self, log_dir: Path) -> None:
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = log_dir / f"ra6m5_upper_console_{stamp}.log"
        self.lock = threading.Lock()

    def write(self, line: str) -> None:
        with self.lock:
            with self.path.open("a", encoding="utf-8") as handle:
                handle.write(line + "\n")


class SerialChannel:
    def __init__(self, name: str, app: "QtUpperConsole") -> None:
        self.name = name
        self.app = app
        self.ser: serial.Serial | None = None
        self.stop_event = threading.Event()

    def open(self, port: str, baud: int) -> None:
        self.close()
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
        self.stop_event.clear()
        threading.Thread(target=self._reader, daemon=True).start()
        self.app.emit_log(self.name, "INFO", f"opened {port} @ {baud}")

    def close(self) -> None:
        self.stop_event.set()
        if self.ser is not None:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None

    def is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def write(self, data: bytes, label: str) -> None:
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError(f"{self.name} serial is not open")
        self.ser.write(data)
        display = data.decode("ascii", errors="ignore").strip() if self.name == "ARM" else bytes_to_hex(data)
        self.app.emit_log(self.name, "TX", f"{label}: {display}")

    def _reader(self) -> None:
        while not self.stop_event.is_set():
            if self.ser is None:
                return
            try:
                data = self.ser.read(256)
            except serial.SerialException as exc:
                self.app.emit_log(self.name, "ERROR", str(exc))
                return
            if data:
                self.app.on_serial_data(self.name, data)


class QtUpperConsole(QMainWindow):
    def __init__(self, log_dir: Path | None = None) -> None:
        super().__init__()
        self.setWindowTitle(APP_TITLE)
        self.resize(1180, 760)
        self.logger = SessionLogger(log_dir or (Path.cwd() / "logs"))
        self.log_queue: queue.Queue[str] = queue.Queue()
        self.arm = SerialChannel("ARM", self)
        self.jetson = SerialChannel("JETSON", self)
        self.vision_stop = threading.Event()
        self.status_labels: dict[str, QLabel] = {}
        self._build_ui()
        self.refresh_ports()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._flush_logs)
        self.timer.start(80)
        self.emit_log("APP", "INFO", f"log file: {self.logger.path}")

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        main = QHBoxLayout(root)
        left = QVBoxLayout()
        right = QVBoxLayout()
        main.addLayout(left, 0)
        main.addLayout(right, 1)
        left.addWidget(self._serial_group())
        left.addWidget(self._arm_group())
        left.addWidget(self._jetson_group())
        left.addStretch(1)
        right.addWidget(self._status_group())
        log_bar = QHBoxLayout()
        log_bar.addWidget(QLabel("运行日志"))
        log_bar.addStretch(1)
        log_bar.addWidget(self._button("清空日志", self.clear_visible_log))
        right.addLayout(log_bar)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        right.addWidget(self.log_text, 1)

    def _serial_group(self) -> QGroupBox:
        group = QGroupBox("串口连接")
        grid = QGridLayout(group)
        self.arm_port = QComboBox()
        self.jetson_port = QComboBox()
        self.baud_edit = QLineEdit(str(DEFAULT_BAUD))
        grid.addWidget(QLabel("ARM控制口"), 0, 0)
        grid.addWidget(self.arm_port, 0, 1)
        grid.addWidget(self._button("打开", self.open_arm), 0, 2)
        grid.addWidget(self._button("关闭", lambda: self.close_channel(self.arm)), 0, 3)
        grid.addWidget(QLabel("Jetson模拟口"), 1, 0)
        grid.addWidget(self.jetson_port, 1, 1)
        grid.addWidget(self._button("打开", self.open_jetson), 1, 2)
        grid.addWidget(self._button("关闭", lambda: self.close_channel(self.jetson)), 1, 3)
        grid.addWidget(QLabel("波特率"), 2, 0)
        grid.addWidget(self.baud_edit, 2, 1)
        grid.addWidget(self._button("刷新串口", self.refresh_ports), 2, 2)
        grid.addWidget(self._button("安全停止", self.safety_stop), 2, 3)
        return group

    def _arm_group(self) -> QGroupBox:
        group = QGroupBox("机械臂控制")
        layout = QVBoxLayout(group)
        grid = QGridLayout()
        commands = ["soft_reset", "read_all", "target_enable", "target_disable", "laser_on", "laser_off", "gripper_open", "gripper_stop"]
        for idx, command in enumerate(commands):
            grid.addWidget(self._button(command, lambda c=command: self.send_arm_command(c)), idx // 2, idx % 2)
        layout.addLayout(grid)

        self.auto_x = QLineEdit("0")
        self.auto_y = QLineEdit("-50")
        self.auto_z = QLineEdit("0")
        row = QHBoxLayout()
        row.addWidget(QLabel("auto"))
        row.addWidget(self.auto_x)
        row.addWidget(self.auto_y)
        row.addWidget(self.auto_z)
        row.addWidget(self._button("发送", self.send_auto))
        layout.addLayout(row)

        self.joint_id = QLineEdit("0")
        self.joint_angle = QLineEdit("0")
        row = QHBoxLayout()
        row.addWidget(QLabel("关节"))
        row.addWidget(self.joint_id)
        row.addWidget(self.joint_angle)
        row.addWidget(self._button("abs", lambda: self.send_joint("abs_rotate")))
        row.addWidget(self._button("rel", lambda: self.send_joint("rel_rotate")))
        layout.addLayout(row)

        self.circle_time = QLineEdit("8000")
        self.circle_radius = QLineEdit("5")
        row = QHBoxLayout()
        row.addWidget(QLabel("画圆"))
        row.addWidget(self.circle_time)
        row.addWidget(self.circle_radius)
        row.addWidget(self._button("发送", self.send_circle))
        layout.addLayout(row)

        self.custom_cmd = QLineEdit()
        row = QHBoxLayout()
        row.addWidget(self.custom_cmd)
        row.addWidget(self._button("发送自定义", lambda: self.send_arm_command(self.custom_cmd.text())))
        layout.addLayout(row)

        self.arm_history = QComboBox()
        self.arm_history.setEditable(True)
        self.arm_history.setMaxVisibleItems(ARM_HISTORY_LIMIT)
        self.arm_history.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
        if self.arm_history.lineEdit() is not None:
            self.arm_history.lineEdit().setPlaceholderText("输入或选择最近15条ARM指令")
        row = QHBoxLayout()
        row.addWidget(QLabel("历史"))
        row.addWidget(self.arm_history)
        row.addWidget(self._button("发送历史", self.send_history_command))
        layout.addLayout(row)
        return group

    def _jetson_group(self) -> QGroupBox:
        group = QGroupBox("Jetson 模拟器")
        layout = QVBoxLayout(group)
        row = QHBoxLayout()
        row.addWidget(self._button("启动状态机 AA 01 01 BB", lambda: self.send_target_ctrl(True)))
        row.addWidget(self._button("关闭状态机 AA 01 00 BB", lambda: self.send_target_ctrl(False)))
        layout.addLayout(row)
        self.dcx = QLineEdit("0")
        self.dcy = QLineEdit("0")
        self.period_ms = QLineEdit("200")
        row = QHBoxLayout()
        row.addWidget(QLabel("dcx"))
        row.addWidget(self.dcx)
        row.addWidget(QLabel("dcy"))
        row.addWidget(self.dcy)
        row.addWidget(QLabel("ms"))
        row.addWidget(self.period_ms)
        layout.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(self._button("单发视觉误差", self.send_vision_once))
        row.addWidget(self._button("开始周期发送", self.start_periodic_vision))
        row.addWidget(self._button("停止周期发送", self.stop_periodic_vision))
        layout.addLayout(row)
        self.sequence = QLineEdit("15,-10:2;8,-5:1;0,0:0")
        row = QHBoxLayout()
        row.addWidget(self.sequence)
        row.addWidget(self._button("运行序列", self.run_vision_sequence))
        layout.addLayout(row)
        return group

    def _status_group(self) -> QGroupBox:
        group = QGroupBox("状态")
        grid = QGridLayout(group)
        keys = ["ARM_PORT", "JETSON_PORT", "POSE_VALID", "TARGET_CTRL", "READY", "ALIGN_DONE", "OUTPUT", "ERROR"]
        for idx, key in enumerate(keys):
            label = QLabel(f"{key}: -")
            label.setMinimumWidth(190)
            self.status_labels[key] = label
            grid.addWidget(label, idx // 4, idx % 4)
        return group

    def _button(self, text: str, callback) -> QPushButton:
        button = QPushButton(text)
        button.clicked.connect(lambda _checked=False, cb=callback: cb())
        return button

    def refresh_ports(self) -> None:
        ports = [item.device for item in list_ports.comports()]
        for combo, preferred in ((self.arm_port, "COM7"), (self.jetson_port, "COM14")):
            current = combo.currentText()
            combo.clear()
            combo.addItems(ports)
            idx = combo.findText(current) if current else -1
            if idx < 0:
                idx = combo.findText(preferred)
            if idx >= 0:
                combo.setCurrentIndex(idx)
        if ports:
            self.emit_log("APP", "INFO", "已刷新串口: " + ", ".join(ports))
        else:
            self.emit_log("APP", "WARN", "未发现串口，请检查USB连接后再点“刷新串口”。")

    def open_arm(self) -> None:
        self._open_channel(self.arm, self.arm_port.currentText(), "ARM_PORT")

    def open_jetson(self) -> None:
        self._open_channel(self.jetson, self.jetson_port.currentText(), "JETSON_PORT")

    def _open_channel(self, channel: SerialChannel, port: str, key: str) -> None:
        if not port:
            self.emit_log(channel.name, "ERROR", "未选择串口，请先点“刷新串口”。")
            QMessageBox.warning(self, "串口未选择", "未选择串口，请先点“刷新串口”。")
            return
        try:
            channel.open(port, int(self.baud_edit.text()))
            self._set_status(key, f"已打开 {port}")
        except Exception as exc:
            self.emit_log(channel.name, "ERROR", str(exc))
            QMessageBox.warning(self, "串口打开失败", str(exc))

    def close_channel(self, channel: SerialChannel) -> None:
        if channel is self.jetson:
            self.stop_periodic_vision()
        channel.close()
        self._set_status("ARM_PORT" if channel is self.arm else "JETSON_PORT", "已关闭")
        self.emit_log(channel.name, "INFO", "已关闭串口")

    def send_arm_command(self, command: str) -> None:
        if not isinstance(command, str):
            command = str(command)
        command = command.strip()
        if not command:
            self.emit_log("ARM", "WARN", "空命令已忽略。")
            return
        self._remember_arm_command(command)
        if not self.arm.is_open():
            self.emit_log("ARM", "ERROR", f"ARM串口未打开，命令未发送: {command}")
            return
        try:
            self.arm.write(build_arm_command(command), f"发送命令 {command}")
        except Exception as exc:
            self.emit_log("ARM", "ERROR", str(exc))

    def send_auto(self) -> None:
        self.send_arm_command(f"auto {self.auto_x.text()} {self.auto_y.text()} {self.auto_z.text()}")

    def send_joint(self, command: str) -> None:
        self.send_arm_command(f"{command} {self.joint_id.text()} {self.joint_angle.text()}")

    def send_circle(self) -> None:
        self.send_arm_command(f"circle {self.circle_time.text()} {self.circle_radius.text()}")

    def send_history_command(self) -> None:
        self.send_arm_command(self.arm_history.currentText())

    def _remember_arm_command(self, command: str) -> None:
        history = [self.arm_history.itemText(idx) for idx in range(self.arm_history.count())]
        history = [item for item in history if item != command]
        history.insert(0, command)
        history = history[:ARM_HISTORY_LIMIT]
        self.arm_history.blockSignals(True)
        self.arm_history.clear()
        self.arm_history.addItems(history)
        self.arm_history.setCurrentIndex(0)
        if self.arm_history.lineEdit() is not None:
            self.arm_history.setEditText(command)
        self.arm_history.blockSignals(False)

    def send_target_ctrl(self, enable: bool) -> None:
        if not self.jetson.is_open():
            self.emit_log("JETSON", "ERROR", "Jetson模拟串口未打开，控制帧未发送。")
            return
        try:
            self.jetson.write(build_target_control_frame(enable), "TARGET_CTRL_START" if enable else "TARGET_CTRL_STOP")
        except Exception as exc:
            self.emit_log("JETSON", "ERROR", str(exc))

    def send_vision_once(self) -> None:
        try:
            self.send_vision_values(int(self.dcx.text()), int(self.dcy.text()))
        except Exception as exc:
            self.emit_log("JETSON", "ERROR", str(exc))

    def send_vision_values(self, dcx: int, dcy: int) -> None:
        if not self.jetson.is_open():
            self.emit_log("JETSON", "ERROR", f"Jetson模拟串口未打开，视觉误差未发送: dcx={dcx} dcy={dcy}")
            return
        self.jetson.write(build_vision_error_frame(dcx, dcy), f"VISION dcx={dcx} dcy={dcy}")

    def start_periodic_vision(self) -> None:
        self.vision_stop.clear()
        dcx = int(self.dcx.text())
        dcy = int(self.dcy.text())
        period = max(20, int(self.period_ms.text()))
        threading.Thread(target=self._vision_loop, args=(dcx, dcy, period), daemon=True).start()
        self.emit_log("JETSON", "INFO", "periodic vision started")

    def stop_periodic_vision(self) -> None:
        self.vision_stop.set()
        self.emit_log("JETSON", "INFO", "已停止周期视觉发送")

    def _vision_loop(self, dcx: int, dcy: int, period_ms: int) -> None:
        while not self.vision_stop.is_set():
            try:
                self.send_vision_values(dcx, dcy)
            except Exception as exc:
                self.emit_log("JETSON", "ERROR", str(exc))
                self.vision_stop.set()
                return
            time.sleep(period_ms / 1000.0)

    def run_vision_sequence(self) -> None:
        try:
            sequence = parse_vision_sequence(self.sequence.text())
            period = max(20, int(self.period_ms.text()))
        except Exception as exc:
            self.emit_log("JETSON", "ERROR", str(exc))
            return
        self.vision_stop.clear()
        threading.Thread(target=self._sequence_loop, args=(sequence, period), daemon=True).start()
        self.emit_log("JETSON", "INFO", f"vision sequence started: {self.sequence.text()}")

    def _sequence_loop(self, sequence: list[tuple[int, int, int]], period_ms: int) -> None:
        for dcx, dcy, count in sequence:
            if self.vision_stop.is_set():
                return
            if count == 0:
                while not self.vision_stop.is_set():
                    self.send_vision_values(dcx, dcy)
                    time.sleep(period_ms / 1000.0)
                return
            for _ in range(count):
                if self.vision_stop.is_set():
                    return
                self.send_vision_values(dcx, dcy)
                time.sleep(period_ms / 1000.0)
        self.vision_stop.set()

    def safety_stop(self) -> None:
        self.stop_periodic_vision()
        self.send_arm_command("target_disable")
        self.send_arm_command("laser_off")
        self.emit_log("APP", "SAFETY", "已停止视觉发送，并发送 target_disable / laser_off")

    def on_serial_data(self, source: str, data: bytes) -> None:
        if source == "ARM":
            text = data.decode("utf-8", errors="replace")
            for line in text.replace("\r", "\n").split("\n"):
                line = line.strip()
                if line:
                    self.emit_log("ARM", "RX", line)
                    self._update_arm_status(line)
        else:
            self.emit_log("JETSON", "RX", bytes_to_hex(data))
            for status in parse_ra6_status_frames(data):
                self.emit_log("JETSON", "STATUS", f"{status.name}: {bytes_to_hex(status.raw)}")
                self._update_ra6_status(status.name)

    def _update_arm_status(self, line: str) -> None:
        lower = line.lower()
        if "soft reset final verify pass" in lower:
            self._set_status("POSE_VALID", "YES")
        elif "pose invalid" in lower or "final fail" in lower:
            self._set_status("POSE_VALID", "NO")

    def _update_ra6_status(self, name: str) -> None:
        if name == "TARGET_CTRL_ON":
            self._set_status("TARGET_CTRL", "ON")
        elif name == "TARGET_CTRL_OFF":
            self._set_status("TARGET_CTRL", "OFF")
            self._set_status("OUTPUT", "OFF")
        elif name == "READY":
            self._set_status("READY", "YES")
        elif name == "ALIGN_DONE":
            self._set_status("ALIGN_DONE", "YES")
        elif name == "OUTPUT_ON":
            self._set_status("OUTPUT", "ON")
        elif name == "OUTPUT_OFF":
            self._set_status("OUTPUT", "OFF")
        elif name == "ERROR":
            self._set_status("ERROR", "YES")

    def _set_status(self, key: str, value: str) -> None:
        self.status_labels[key].setText(f"{key}: {value}")

    def clear_visible_log(self) -> None:
        self.log_text.clear()

    def emit_log(self, source: str, kind: str, message: str) -> None:
        line = f"[{dt.datetime.now().strftime('%H:%M:%S.%f')[:-3]}] [{source}-{kind}] {message}"
        self.logger.write(line)
        self.log_queue.put(line)

    def _flush_logs(self) -> None:
        while True:
            try:
                line = self.log_queue.get_nowait()
            except queue.Empty:
                return
            self.log_text.append(line)

    def closeEvent(self, event) -> None:  # noqa: N802
        self.stop_periodic_vision()
        self.arm.close()
        self.jetson.close()
        event.accept()


def qt_gui_self_test() -> None:
    app = QApplication.instance() or QApplication([])
    window = QtUpperConsole(Path("logs"))
    assert window.windowTitle() == APP_TITLE
    assert "ARM_PORT" in window.status_labels
    for button in window.findChildren(QPushButton):
        if button.text() == "soft_reset":
            button.click()
            break
    else:
        raise AssertionError("soft_reset button not found")
    window._flush_logs()
    log_text = window.log_text.toPlainText()
    assert "soft_reset" in log_text
    assert "False" not in log_text
    window.close()
    app.processEvents()


def main() -> int:
    parser = argparse.ArgumentParser(description=APP_TITLE)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--gui-self-test", action="store_true")
    parser.add_argument("--log-dir", type=Path, default=Path.cwd() / "logs")
    args = parser.parse_args()
    if args.self_test:
        protocol_self_test()
        print("upper console self-test passed")
        return 0
    if args.gui_self_test:
        protocol_self_test()
        qt_gui_self_test()
        print("upper console gui self-test passed")
        return 0
    app = QApplication([])
    window = QtUpperConsole(args.log_dir)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
