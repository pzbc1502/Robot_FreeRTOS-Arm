from __future__ import annotations

from dataclasses import dataclass

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


def self_test() -> None:
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
