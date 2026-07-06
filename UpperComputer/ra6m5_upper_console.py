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
from PySide6.QtCore import QSettings, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

APP_TITLE = "RA6M5 Robot Serial Console"
DEFAULT_BAUD = 115200
ARM_HISTORY_LIMIT = 15
SETTINGS_ORG = "RA6M5Robot"
SETTINGS_APP = "UpperConsole"
DEFAULT_DEMO_SEQUENCE = "30,-20:5;15,-10:5;8,-5:5;0,0:0"
DEFAULT_DEMO_PERIOD_MS = "200"
DEFAULT_CAPTURE_RECIPES = {
    "左视图": "# 左视图\nsoft_reset\nauto 45 -130 -15\nabs_rotate 0 85\nread_all",
    "正视图": "# 正视图\nsoft_reset\nauto 0 -130 -15\nabs_rotate 0 90\nread_all",
    "右视图": "# 右视图\nsoft_reset\nauto -45 -130 -15\nabs_rotate 0 105\nread_all",
}
CAPTURE_RECIPE_KEYS = {"左视图": "left", "正视图": "front", "右视图": "right"}
JETSON_SOF = 0xFF
JETSON_EOF = 0xFE
JETSON_LEN = 0x05
JETSON_FUNC_VISION_ERROR = 0x03
JETSON_CTRL_SOF = 0xAA
JETSON_CTRL_EOF = 0xBB
JETSON_FUNC_TARGET_CTRL = 0x01
JETSON_PROTOCOL_OLD = "old"
JETSON_PROTOCOL_NEW = "new"
JETSON_UNIFIED_SOF0 = 0xA5
JETSON_UNIFIED_SOF1 = 0x5A
JETSON_UNIFIED_VERSION = 0x01
JETSON_MSG_HEARTBEAT = 0x01
JETSON_MSG_TARGET_CTRL = 0x02
JETSON_MSG_VISION_ERROR = 0x03
JETSON_MSG_STATUS = 0x81
JETSON_MSG_ERROR = 0xFE
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


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_unified_frame(msg_type: int, seq: int, payload: bytes) -> bytes:
    if len(payload) > 255:
        raise ValueError("unified payload too large")
    body = bytes([JETSON_UNIFIED_VERSION, msg_type & 0xFF, seq & 0xFF, len(payload)]) + payload
    crc = crc16_modbus(body)
    return bytes([JETSON_UNIFIED_SOF0, JETSON_UNIFIED_SOF1]) + body + crc.to_bytes(2, "little")


def build_unified_heartbeat_frame(seq: int, tick_ms: int) -> bytes:
    return build_unified_frame(JETSON_MSG_HEARTBEAT, seq, int(tick_ms & 0xFFFFFFFF).to_bytes(4, "little"))


def build_unified_target_control_frame(enable: bool, seq: int) -> bytes:
    return build_unified_frame(JETSON_MSG_TARGET_CTRL, seq, bytes([0x01 if enable else 0x00]))


def build_unified_vision_error_frame(dcx: int, dcy: int, seq: int) -> bytes:
    return build_unified_frame(JETSON_MSG_VISION_ERROR, seq, _int16_le(dcx) + _int16_le(dcy) + b"\x01")


def should_log_serial_tx(label: str) -> bool:
    return not label.startswith("HEARTBEAT")


def should_display_arm_line(line: str) -> bool:
    return "[JETSON_RX] heartbeat seq=" not in line


def _status_from_func_value(raw: bytes, func: int, value: int) -> Ra6Status:
    return Ra6Status(raw=raw, func=func, value=value, name=STATUS_NAMES.get((func, value), "UNKNOWN"))


def parse_ra6_status_stream(data: bytes) -> tuple[list[Ra6Status], bytes]:
    frames: list[Ra6Status] = []
    idx = 0
    consumed = 0
    while idx < len(data):
        if idx + 3 < len(data) and data[idx] == RA6_SOF:
            raw = data[idx:idx + 4]
            if raw[3] == RA6_EOF:
                frames.append(_status_from_func_value(raw, raw[1], raw[2]))
                idx += 4
                consumed = idx
                continue
        if idx + 7 < len(data) and data[idx] == JETSON_UNIFIED_SOF0 and data[idx + 1] == JETSON_UNIFIED_SOF1:
            if data[idx + 2] != JETSON_UNIFIED_VERSION:
                idx += 1
                consumed = idx
                continue
            payload_len = data[idx + 5]
            total_len = 8 + payload_len
            if idx + total_len > len(data):
                break
            raw = data[idx:idx + total_len]
            body = raw[2:6 + payload_len]
            rx_crc = int.from_bytes(raw[6 + payload_len:8 + payload_len], "little")
            if crc16_modbus(body) != rx_crc:
                idx += 1
                consumed = idx
                continue
            msg_type = raw[3]
            payload = raw[6:6 + payload_len]
            if msg_type == JETSON_MSG_STATUS and len(payload) >= 3:
                frames.append(_status_from_func_value(raw, payload[0], payload[1]))
            elif msg_type == JETSON_MSG_ERROR and len(payload) >= 1:
                frames.append(Ra6Status(raw=raw, func=0xFE, value=payload[0], name="ERROR"))
            idx += total_len
            consumed = idx
            continue
        idx += 1
        consumed = idx
    return frames, data[consumed:]


def parse_ra6_status_frames(data: bytes) -> list[Ra6Status]:
    return parse_ra6_status_stream(data)[0]


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
    assert build_unified_heartbeat_frame(1, 1234) == bytes.fromhex("A5 5A 01 01 01 04 D2 04 00 00 19 AF")
    assert build_unified_target_control_frame(True, 2) == bytes.fromhex("A5 5A 01 02 02 01 01 79 E8")
    assert build_unified_vision_error_frame(-7, -50, 3) == bytes.fromhex("A5 5A 01 03 03 05 F9 FF CE FF 01 39 EF")
    assert build_arm_command(" soft_reset ") == b"soft_reset\r\n"
    frames = parse_ra6_status_frames(bytes.fromhex("00 CC 04 01 DD 99 CC 03 00 DD"))
    assert [(item.func, item.value, item.name) for item in frames] == [
        (0x04, 0x01, "TARGET_CTRL_ON"),
        (0x03, 0x00, "OUTPUT_OFF"),
    ]
    unified = build_unified_frame(0x81, 7, bytes([0x02, 0x01, 0x00]))
    assert [(item.func, item.value, item.name) for item in parse_ra6_status_frames(unified)] == [
        (0x02, 0x01, "ALIGN_DONE"),
    ]
    bad_crc = bytearray(unified)
    bad_crc[-1] ^= 0xFF
    assert parse_ra6_status_frames(bytes(bad_crc)) == []
    assert parse_vision_sequence("15,-10:2;8,-5:1;0,0:0") == [(15, -10, 2), (8, -5, 1), (0, 0, 0)]
    assert should_log_serial_tx("HEARTBEAT") is False
    assert should_log_serial_tx("TARGET_CTRL_START_NEW") is True
    assert should_display_arm_line("[JETSON_RX] heartbeat seq=84") is False
    assert should_display_arm_line("[JETSON_RX] target_ctrl=1") is True


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
        self.write_lock = threading.Lock()

    def open(self, port: str, baud: int) -> None:
        self.close()
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.05, write_timeout=1.0)
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
        with self.write_lock:
            self.ser.write(data)
        if should_log_serial_tx(label):
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
        self.resize(1500, 900)
        self.settings = QSettings(SETTINGS_ORG, SETTINGS_APP)
        self.logger = SessionLogger(log_dir or (Path.cwd() / "logs"))
        self.log_queue: queue.Queue[str] = queue.Queue()
        self.arm = SerialChannel("ARM", self)
        self.jetson = SerialChannel("JETSON", self)
        self.vision_stop = threading.Event()
        self.heartbeat_stop = threading.Event()
        self.heartbeat_stop.set()
        self.heartbeat_thread: threading.Thread | None = None
        self.demo_stop = threading.Event()
        self.ra6_events = {name: threading.Event() for name in STATUS_NAMES.values()}
        self.capture_step_index = {name: 0 for name in DEFAULT_CAPTURE_RECIPES}
        self.status_labels: dict[str, QLabel] = {}
        self.status_dots: dict[str, QLabel] = {}
        self.unified_seq = 0
        self.jetson_rx_buffer = bytearray()
        self._build_ui()
        self._set_status("PROTO", self.current_protocol_label())
        self._set_status("HEARTBEAT", "OFF")
        self._update_protocol_widgets()
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
        self.left_tabs = QTabWidget()
        self.left_tabs.addTab(self._arm_group(), "机械臂控制")
        self.left_tabs.addTab(self._jetson_group(), "Jetson/定靶")
        self.left_tabs.addTab(self._capture_group(), "拍摄点位")
        left.addWidget(self.left_tabs, 1)
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
        self.protocol_mode = QComboBox()
        self.protocol_mode.addItem("OLD AA/FF/CC", JETSON_PROTOCOL_OLD)
        self.protocol_mode.addItem("NEW A5 5A + CRC16", JETSON_PROTOCOL_NEW)
        saved_protocol = self._settings_text("jetson/protocol", JETSON_PROTOCOL_OLD)
        protocol_index = self.protocol_mode.findData(saved_protocol)
        self.protocol_mode.setCurrentIndex(protocol_index if protocol_index >= 0 else 0)
        self.protocol_mode.currentIndexChanged.connect(self.on_protocol_changed)

        self.heartbeat_enable = QCheckBox("Heartbeat keep")
        self.heartbeat_enable.setChecked(self._settings_bool("jetson/heartbeat_enable", False))
        self.heartbeat_enable.toggled.connect(self.on_heartbeat_toggled)
        self.heartbeat_period_ms = QLineEdit(self._settings_text("jetson/heartbeat_ms", "200"))

        row = QHBoxLayout()
        row.addWidget(QLabel("Protocol"))
        row.addWidget(self.protocol_mode, 1)
        row.addWidget(self.heartbeat_enable)
        row.addWidget(QLabel("HB ms"))
        row.addWidget(self.heartbeat_period_ms)
        layout.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(self._button("启动状态机 AA 01 01 BB", lambda: self.send_target_ctrl(True)))
        row.addWidget(self._button("关闭状态机 AA 01 00 BB", self.stop_target_ctrl_with_reset))
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
        layout.addWidget(self._target_demo_group())
        return group

    def _target_demo_group(self) -> QGroupBox:
        group = QGroupBox("一键定靶演示")
        layout = QVBoxLayout(group)
        self.demo_sequence = QLineEdit(self._settings_text("demo/sequence", DEFAULT_DEMO_SEQUENCE))
        self.demo_period_ms = QLineEdit(self._settings_text("demo/period_ms", DEFAULT_DEMO_PERIOD_MS))
        row = QHBoxLayout()
        row.addWidget(QLabel("序列"))
        row.addWidget(self.demo_sequence, 1)
        row.addWidget(QLabel("ms"))
        row.addWidget(self.demo_period_ms)
        layout.addLayout(row)

        self.demo_send_enable = QCheckBox("先启动状态机")
        self.demo_wait_ready = QCheckBox("等待READY")
        self.demo_prompt_p000 = QCheckBox("对准后提示按P000")
        self.demo_auto_stop = QCheckBox("输出关闭后自动停止")
        for checkbox, key, default in (
            (self.demo_send_enable, "demo/send_enable", True),
            (self.demo_wait_ready, "demo/wait_ready", True),
            (self.demo_prompt_p000, "demo/prompt_p000", True),
            (self.demo_auto_stop, "demo/auto_stop", True),
        ):
            checkbox.setChecked(self._settings_bool(key, default))
        row = QHBoxLayout()
        for checkbox in (self.demo_send_enable, self.demo_wait_ready, self.demo_prompt_p000, self.demo_auto_stop):
            row.addWidget(checkbox)
        layout.addLayout(row)

        row = QHBoxLayout()
        row.addWidget(self._button("一键定靶演示", self.start_target_demo))
        row.addWidget(self._button("停止演示", self.stop_target_demo))
        row.addWidget(self._button("保存演示配置", self.save_demo_settings))
        layout.addLayout(row)
        return group

    def _capture_group(self) -> QGroupBox:
        group = QGroupBox("人脸三视图拍摄点位")
        layout = QVBoxLayout(group)

        self.capture_view = QComboBox()
        self.capture_view.addItems(DEFAULT_CAPTURE_RECIPES.keys())
        self.capture_view.currentTextChanged.connect(self.load_capture_recipe)
        row = QHBoxLayout()
        row.addWidget(QLabel("视图"))
        row.addWidget(self.capture_view, 1)
        layout.addLayout(row)

        self.capture_recipe = QTextEdit()
        self.capture_recipe.setMinimumHeight(180)
        layout.addWidget(self.capture_recipe, 1)

        row = QHBoxLayout()
        row.addWidget(self._button("保存配方", self.save_capture_recipe))
        row.addWidget(self._button("重置默认", self.reset_capture_recipe))
        row.addWidget(self._button("从头开始", self.reset_capture_steps))
        row.addWidget(self._button("发送下一条", self.send_next_capture_command))
        layout.addLayout(row)

        self.load_capture_recipe(self.capture_view.currentText())
        return group

    def _status_group(self) -> QGroupBox:
        group = QGroupBox("状态")
        grid = QGridLayout(group)
        keys = ["ARM_PORT", "JETSON_PORT", "PROTO", "HEARTBEAT", "POSE_VALID", "TARGET_CTRL", "READY", "ALIGN_DONE", "CONFIRMING", "OUTPUT", "ERROR"]
        for idx, key in enumerate(keys):
            dot = QLabel()
            dot.setFixedSize(14, 14)
            self.status_dots[key] = dot
            self._set_status_dot(key, "gray")
            label = QLabel(f"{key}: -")
            label.setMinimumWidth(190)
            self.status_labels[key] = label
            cell = QHBoxLayout()
            cell.addWidget(dot)
            cell.addWidget(label)
            wrapper = QWidget()
            wrapper.setLayout(cell)
            grid.addWidget(wrapper, idx // 3, idx % 3)
        return group

    def _settings_text(self, key: str, default: str) -> str:
        return str(self.settings.value(key, default))

    def _settings_bool(self, key: str, default: bool) -> bool:
        value = self.settings.value(key, default)
        if isinstance(value, bool):
            return value
        return str(value).lower() in {"1", "true", "yes", "on"}

    def _capture_settings_key(self, view: str) -> str:
        return f"capture/{CAPTURE_RECIPE_KEYS.get(view, 'left')}"

    def _button(self, text: str, callback) -> QPushButton:
        button = QPushButton(text)
        button.clicked.connect(lambda _checked=False, cb=callback: cb())
        return button

    def current_jetson_protocol(self) -> str:
        if not hasattr(self, "protocol_mode"):
            return JETSON_PROTOCOL_OLD
        return str(self.protocol_mode.currentData() or JETSON_PROTOCOL_OLD)

    def current_protocol_label(self) -> str:
        return "NEW" if self.current_jetson_protocol() == JETSON_PROTOCOL_NEW else "OLD"

    def _update_protocol_widgets(self) -> None:
        enabled = self.current_jetson_protocol() == JETSON_PROTOCOL_NEW
        self.heartbeat_enable.setEnabled(enabled)
        self.heartbeat_period_ms.setEnabled(enabled)

    def _next_unified_seq(self) -> int:
        self.unified_seq = (self.unified_seq + 1) & 0xFF
        return self.unified_seq

    def on_protocol_changed(self) -> None:
        protocol = self.current_jetson_protocol()
        self.settings.setValue("jetson/protocol", protocol)
        self.settings.sync()
        self._set_status("PROTO", self.current_protocol_label())
        self._update_protocol_widgets()
        if protocol == JETSON_PROTOCOL_NEW:
            self.emit_log("JETSON", "INFO", "protocol switched to NEW A5 5A + CRC16")
            if self.heartbeat_enable.isChecked():
                self.start_heartbeat()
        else:
            self.stop_heartbeat()
            self.emit_log("JETSON", "INFO", "protocol switched to OLD AA/FF/CC; heartbeat is disabled")

    def on_heartbeat_toggled(self, checked: bool) -> None:
        self.settings.setValue("jetson/heartbeat_enable", checked)
        self.settings.setValue("jetson/heartbeat_ms", self.heartbeat_period_ms.text())
        self.settings.sync()
        if checked:
            self.start_heartbeat()
        else:
            self.stop_heartbeat()

    def start_heartbeat(self) -> None:
        if self.current_jetson_protocol() != JETSON_PROTOCOL_NEW:
            self._set_status("HEARTBEAT", "OFF")
            self.emit_log("JETSON", "INFO", "OLD protocol has no heartbeat")
            return
        if not self.jetson.is_open():
            self._set_status("HEARTBEAT", "OFF")
            self.emit_log("JETSON", "WARN", "Jetson serial is not open; heartbeat not started")
            return
        try:
            period_ms = max(50, int(self.heartbeat_period_ms.text()))
        except ValueError:
            period_ms = 200
            self.heartbeat_period_ms.setText(str(period_ms))
        self.settings.setValue("jetson/heartbeat_ms", str(period_ms))
        self.settings.sync()

        if self.heartbeat_thread is not None and self.heartbeat_thread.is_alive() and not self.heartbeat_stop.is_set():
            self._set_status("HEARTBEAT", "ON")
            return

        self.heartbeat_stop.set()
        stop_event = threading.Event()
        self.heartbeat_stop = stop_event
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, args=(period_ms, stop_event), daemon=True)
        self.heartbeat_thread.start()
        self._set_status("HEARTBEAT", "ON")
        self.emit_log("JETSON", "INFO", f"heartbeat started: {period_ms} ms")

    def stop_heartbeat(self) -> None:
        self.heartbeat_stop.set()
        self._set_status("HEARTBEAT", "OFF")
        self.emit_log("JETSON", "INFO", "heartbeat stopped")

    def _heartbeat_loop(self, period_ms: int, stop_event: threading.Event) -> None:
        while not stop_event.is_set():
            if not self.jetson.is_open() or self.current_jetson_protocol() != JETSON_PROTOCOL_NEW:
                stop_event.set()
                break
            tick_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
            try:
                self.jetson.write(build_unified_heartbeat_frame(self._next_unified_seq(), tick_ms), "HEARTBEAT")
            except Exception as exc:
                self.emit_log("JETSON", "ERROR", str(exc))
                stop_event.set()
                break
            time.sleep(period_ms / 1000.0)
        if self.heartbeat_stop is stop_event:
            self._set_status("HEARTBEAT", "OFF")

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
            if channel is self.arm:
                self._clear_arm_runtime_status()
            elif channel is self.jetson:
                self._clear_target_runtime_status(clear_target=True)
                if self.current_jetson_protocol() == JETSON_PROTOCOL_NEW and self.heartbeat_enable.isChecked():
                    self.start_heartbeat()
        except Exception as exc:
            self.emit_log(channel.name, "ERROR", str(exc))
            QMessageBox.warning(self, "串口打开失败", str(exc))

    def close_channel(self, channel: SerialChannel) -> None:
        if channel is self.jetson:
            self.stop_periodic_vision()
            self.stop_heartbeat()
        channel.close()
        self._set_status("ARM_PORT" if channel is self.arm else "JETSON_PORT", "已关闭")
        if channel is self.arm:
            self._clear_arm_runtime_status()
        elif channel is self.jetson:
            self._clear_target_runtime_status(clear_target=True)
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

    def send_target_ctrl(self, enable: bool) -> bool:
        if not self.jetson.is_open():
            self.emit_log("JETSON", "ERROR", "Jetson模拟串口未打开，控制帧未发送。")
            return False
        try:
            if self.current_jetson_protocol() == JETSON_PROTOCOL_NEW:
                if enable and self.heartbeat_enable.isChecked():
                    self.start_heartbeat()
                frame = build_unified_target_control_frame(enable, self._next_unified_seq())
                label = "TARGET_CTRL_START_NEW" if enable else "TARGET_CTRL_STOP_NEW"
            else:
                frame = build_target_control_frame(enable)
                label = "TARGET_CTRL_START" if enable else "TARGET_CTRL_STOP"
            self.jetson.write(frame, label)
        except Exception as exc:
            self.emit_log("JETSON", "ERROR", str(exc))
            return False
        return True

    def stop_target_ctrl_with_reset(self) -> None:
        self.stop_heartbeat()
        if not self.send_target_ctrl(False):
            self.send_arm_command("soft_reset")

    def send_vision_once(self) -> None:
        try:
            self.send_vision_values(int(self.dcx.text()), int(self.dcy.text()))
        except Exception as exc:
            self.emit_log("JETSON", "ERROR", str(exc))

    def send_vision_values(self, dcx: int, dcy: int) -> None:
        if not self.jetson.is_open():
            self.emit_log("JETSON", "ERROR", f"Jetson模拟串口未打开，视觉误差未发送: dcx={dcx} dcy={dcy}")
            return
        if self.current_jetson_protocol() == JETSON_PROTOCOL_NEW:
            frame = build_unified_vision_error_frame(dcx, dcy, self._next_unified_seq())
            label = f"VISION_NEW dcx={dcx} dcy={dcy}"
        else:
            frame = build_vision_error_frame(dcx, dcy)
            label = f"VISION dcx={dcx} dcy={dcy}"
        self.jetson.write(frame, label)

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

    def save_demo_settings(self) -> None:
        self.settings.setValue("demo/sequence", self.demo_sequence.text())
        self.settings.setValue("demo/period_ms", self.demo_period_ms.text())
        self.settings.setValue("demo/send_enable", self.demo_send_enable.isChecked())
        self.settings.setValue("demo/wait_ready", self.demo_wait_ready.isChecked())
        self.settings.setValue("demo/prompt_p000", self.demo_prompt_p000.isChecked())
        self.settings.setValue("demo/auto_stop", self.demo_auto_stop.isChecked())
        self.settings.sync()
        self.emit_log("DEMO", "INFO", "演示配置已保存")

    def load_capture_recipe(self, view: str) -> None:
        text = self._settings_text(self._capture_settings_key(view), DEFAULT_CAPTURE_RECIPES.get(view, ""))
        self.capture_recipe.setPlainText(text)
        self.capture_step_index[view] = 0

    def save_capture_recipe(self) -> None:
        view = self.capture_view.currentText()
        self.settings.setValue(self._capture_settings_key(view), self.capture_recipe.toPlainText())
        self.settings.sync()
        self.capture_step_index[view] = 0
        self.emit_log("APP", "INFO", f"{view} 拍摄配方已保存")

    def reset_capture_recipe(self) -> None:
        view = self.capture_view.currentText()
        self.capture_recipe.setPlainText(DEFAULT_CAPTURE_RECIPES.get(view, ""))
        self.capture_step_index[view] = 0
        self.emit_log("APP", "INFO", f"{view} 拍摄配方已重置为默认")

    def _capture_commands(self) -> list[str]:
        commands: list[str] = []
        for line in self.capture_recipe.toPlainText().splitlines():
            command = line.strip()
            if command and not command.startswith("#"):
                commands.append(command)
        return commands

    def reset_capture_steps(self) -> None:
        view = self.capture_view.currentText()
        self.capture_step_index[view] = 0
        self.emit_log("APP", "INFO", f"{view} 拍摄配方已从头开始")

    def send_next_capture_command(self) -> None:
        view = self.capture_view.currentText()
        commands = self._capture_commands()
        index = self.capture_step_index.get(view, 0)
        if index >= len(commands):
            self.emit_log("APP", "INFO", f"{view} 拍摄配方已执行完")
            return
        command = commands[index]
        self.capture_step_index[view] = index + 1
        self.emit_log("APP", "INFO", f"{view} 第 {index + 1}/{len(commands)} 条: {command}")
        self.send_arm_command(command)

    def start_target_demo(self) -> None:
        try:
            sequence = parse_vision_sequence(self.demo_sequence.text())
            period = max(20, int(self.demo_period_ms.text()))
        except Exception as exc:
            self.emit_log("DEMO", "ERROR", str(exc))
            return
        if not self.jetson.is_open():
            self.emit_log("DEMO", "ERROR", "Jetson模拟串口未打开，无法开始一键定靶演示。")
            return
        self.save_demo_settings()
        if self.current_jetson_protocol() == JETSON_PROTOCOL_NEW:
            if self.heartbeat_enable.isChecked():
                self.start_heartbeat()
            else:
                self.heartbeat_enable.setChecked(True)
        self._clear_ra6_events()
        self.demo_stop.clear()
        self.vision_stop.set()
        options = (
            self.demo_send_enable.isChecked(),
            self.demo_wait_ready.isChecked(),
            self.demo_prompt_p000.isChecked(),
            self.demo_auto_stop.isChecked(),
        )
        threading.Thread(target=self._target_demo_loop, args=(sequence, period, options), daemon=True).start()
        self.emit_log("DEMO", "INFO", "一键定靶演示开始")

    def stop_target_demo(self) -> None:
        self.demo_stop.set()
        self.stop_periodic_vision()
        self.stop_heartbeat()
        stop_sent = self.send_target_ctrl(False)
        self.send_arm_command("laser_off")
        if not stop_sent:
            self.send_arm_command("soft_reset")
        self.emit_log("DEMO", "INFO", "演示已停止，已关闭视觉发送、激光输出，并请求回 HOME")

    def _target_demo_loop(self, sequence: list[tuple[int, int, int]], period_ms: int, options: tuple[bool, bool, bool, bool]) -> None:
        send_enable, wait_ready, prompt_p000, auto_stop = options
        if send_enable:
            self.send_target_ctrl(True)
        if wait_ready and not self._wait_ra6_event("READY", 12.0):
            self.stop_periodic_vision()
            self.stop_heartbeat()
            self.send_target_ctrl(False)
            self.emit_log("DEMO", "ERROR", "未收到 READY，演示停止。请先确认 soft_reset 和状态机启动。")
            return

        self.vision_stop.clear()
        threading.Thread(target=self._sequence_loop, args=(sequence, period_ms), daemon=True).start()
        self.emit_log("DEMO", "INFO", f"开始发送视觉序列: {self.demo_sequence.text()}")

        if prompt_p000 and self._wait_ra6_event("ALIGN_DONE", 30.0):
            self.emit_log("DEMO", "ACTION", "已对准：请按住 P000 KEY 允许激光输出。")
        if prompt_p000 and self._wait_ra6_event("OUTPUT_ON", 30.0):
            self.emit_log("DEMO", "ACTION", "激光已输出：需要关闭时松开 P000 KEY。")
        if auto_stop and self._wait_ra6_event("OUTPUT_OFF", 60.0):
            self.stop_periodic_vision()
            self.stop_target_ctrl_with_reset()
            self.emit_log("DEMO", "INFO", "检测到 OUTPUT_OFF，演示流程结束。")

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
        self.stop_heartbeat()
        self.send_arm_command("target_disable")
        self.send_arm_command("laser_off")
        self.emit_log("APP", "SAFETY", "已停止视觉发送，并发送 target_disable / laser_off")

    def on_serial_data(self, source: str, data: bytes) -> None:
        if source == "ARM":
            text = data.decode("utf-8", errors="replace")
            for line in text.replace("\r", "\n").split("\n"):
                line = line.strip()
                if line and should_display_arm_line(line):
                    self.emit_log("ARM", "RX", line)
                    self._update_arm_status(line)
        else:
            self.emit_log("JETSON", "RX", bytes_to_hex(data))
            self.jetson_rx_buffer.extend(data)
            frames, remaining = parse_ra6_status_stream(bytes(self.jetson_rx_buffer))
            self.jetson_rx_buffer = bytearray(remaining[-64:])
            for status in frames:
                self.emit_log("JETSON", "STATUS", f"{status.name}: {bytes_to_hex(status.raw)}")
                self._update_ra6_status(status.name)

    def _update_arm_status(self, line: str) -> None:
        lower = line.lower()
        if "soft reset final verify pass" in lower:
            self._set_status("POSE_VALID", "YES")
        elif "pose invalid" in lower or "final fail" in lower:
            self._set_status("POSE_VALID", "NO")
        if "state=confirm" in lower:
            self._set_status("CONFIRMING", "YES", "yellow")
        elif "state=output" in lower or "state=done" in lower or "state=recover" in lower:
            self._set_status("CONFIRMING", "NO")

    def _update_ra6_status(self, name: str) -> None:
        if name in self.ra6_events:
            self.ra6_events[name].set()
        if name == "TARGET_CTRL_ON":
            self._clear_target_runtime_status(clear_target=False)
            self._set_status("TARGET_CTRL", "ON")
        elif name == "TARGET_CTRL_OFF":
            self._set_status("TARGET_CTRL", "OFF")
            self._set_status("READY", "-")
            self._set_status("ALIGN_DONE", "-")
            self._set_status("CONFIRMING", "-")
            self._set_status("OUTPUT", "OFF")
            self._set_status("ERROR", "-")
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

    def _set_status(self, key: str, value: str, color: str | None = None) -> None:
        self.status_labels[key].setText(f"{key}: {value}")
        self._set_status_dot(key, color or self._status_color(key, value))

    def _status_color(self, key: str, value: str) -> str:
        if key == "ERROR" and value == "YES":
            return "red"
        if key == "PROTO":
            return "green" if value == "NEW" else "gray"
        if key == "HEARTBEAT":
            return "green" if value == "ON" else "gray"
        if value in {"YES", "ON"} or value.startswith("已打开"):
            return "green"
        return "gray"

    def _set_status_dot(self, key: str, color: str) -> None:
        colors = {
            "green": "#22c55e",
            "gray": "#9ca3af",
            "red": "#ef4444",
            "yellow": "#f59e0b",
        }
        self.status_dots[key].setStyleSheet(
            f"background-color: {colors[color]}; border-radius: 7px; border: 1px solid #6b7280;"
        )

    def _clear_arm_runtime_status(self) -> None:
        self._set_status("POSE_VALID", "-")

    def _clear_target_runtime_status(self, clear_target: bool) -> None:
        if clear_target:
            self._set_status("TARGET_CTRL", "-")
        for key in ("READY", "ALIGN_DONE", "CONFIRMING", "OUTPUT", "ERROR"):
            self._set_status(key, "-")

    def _clear_ra6_events(self) -> None:
        for event in self.ra6_events.values():
            event.clear()

    def _wait_ra6_event(self, name: str, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        event = self.ra6_events[name]
        while time.monotonic() < deadline and not self.demo_stop.is_set():
            if self.ra6_events["ERROR"].is_set():
                return False
            if event.wait(0.1):
                return True
        return False

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
        self.demo_stop.set()
        self.stop_periodic_vision()
        self.stop_heartbeat()
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
