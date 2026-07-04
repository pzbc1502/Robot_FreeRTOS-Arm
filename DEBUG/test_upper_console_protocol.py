from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from UpperComputer.ra6m5_protocol import (  # noqa: E402
    build_arm_command,
    build_target_control_frame,
    build_vision_error_frame,
    parse_ra6_status_frames,
    parse_vision_sequence,
)


def test_build_vision_error_frame_uses_little_endian_and_checksum() -> None:
    frame = build_vision_error_frame(-7, -50)
    assert frame == bytes.fromhex("FF 05 03 F9 FF CE FF CD FE")


def test_build_target_control_frame() -> None:
    assert build_target_control_frame(True) == bytes.fromhex("AA 01 01 BB")
    assert build_target_control_frame(False) == bytes.fromhex("AA 01 00 BB")


def test_parse_ra6_status_frames_skips_noise() -> None:
    frames = parse_ra6_status_frames(bytes.fromhex("00 CC 04 01 DD 99 CC 03 00 DD"))
    assert [(item.func, item.value, item.name) for item in frames] == [
        (0x04, 0x01, "TARGET_CTRL_ON"),
        (0x03, 0x00, "OUTPUT_OFF"),
    ]


def test_build_arm_command_appends_crlf() -> None:
    assert build_arm_command("soft_reset") == b"soft_reset\r\n"
    assert build_arm_command(" auto 0 -50 0 ") == b"auto 0 -50 0\r\n"


def test_parse_vision_sequence() -> None:
    assert parse_vision_sequence("15,-10:2;8,-5:1;0,0:0") == [
        (15, -10, 2),
        (8, -5, 1),
        (0, 0, 0),
    ]


if __name__ == "__main__":
    test_build_vision_error_frame_uses_little_endian_and_checksum()
    test_build_target_control_frame()
    test_parse_ra6_status_frames_skips_noise()
    test_build_arm_command_appends_crlf()
    test_parse_vision_sequence()
    print("upper console protocol checks passed")
