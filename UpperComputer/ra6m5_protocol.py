from __future__ import annotations

from dataclasses import dataclass

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


def self_test() -> None:
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
