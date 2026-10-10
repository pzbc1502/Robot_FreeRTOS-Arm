"""Execute the compiled FK function on the PC only, never on the real MCU."""

import argparse
import hashlib
import json
from pathlib import Path
import struct
import sys


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE / "python_deps"))
sys.path.insert(0, str(ROOT / "DEBUG"))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import (
    UC_CPU_ARM_CORTEX_M33, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2,
    UC_ARM_REG_R3, UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC,
    UC_ARM_REG_FPEXC,
)
from scan_geometry_preflight import firmware_fk
from readonly_fk_check import layout, rotation_angle


INPUT = 0x2007E000
OUTPUT = 0x2007E100
STACK = 0x2007FFF0
RETURN = 0x000F0000


class BinaryMath:
    def __init__(self, elf_path):
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M33)
        self.uc.mem_map(0, 0x100000)
        self.uc.mem_map(0x20000000, 0x80000)
        self.uc.mem_map(0xE000E000, 0x1000)
        self.uc.mem_write(0xE000ED88, struct.pack("<I", 0xF << 20))
        self.uc.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
        with elf_path.open("rb") as stream:
            elf = ELFFile(stream)
            self.symbols = {s.name: dict(address=s["st_value"], size=s["st_size"])
                            for s in elf.get_section_by_name(".symtab").iter_symbols() if s.name}
            self.robot_layout = layout(elf.get_dwarf_info(), "robot")
            self.joint_layout = layout(elf.get_dwarf_info(), "joint")
            for segment in elf.iter_segments():
                if segment["p_type"] == "PT_LOAD" and segment["p_filesz"]:
                    address = segment["p_vaddr"]
                    if address < 0x100000 or 0x20000000 <= address < 0x20080000:
                        self.uc.mem_write(address, segment.data())
            init = self.symbols["g_joints_init"]
            robot = self.symbols["g_robot"]
            initial = bytes(self.uc.mem_read(init["address"], init["size"]))
            self.uc.mem_write(robot["address"] + self.robot_layout["members"]["joints"], initial)

    def call(self, function, arguments):
        if function not in ("robot_kinematics_forward", "robot_update_current_angle_from_data"):
            raise ValueError("Only pure math functions are permitted in this emulator")
        self.uc.reg_write(UC_ARM_REG_SP, STACK)
        self.uc.reg_write(UC_ARM_REG_LR, RETURN | 1)
        for register, value in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3), arguments):
            self.uc.reg_write(register, value)
        self.uc.emu_start(self.symbols[function]["address"] | 1, RETURN, count=2000000)
        if self.uc.reg_read(UC_ARM_REG_PC) != RETURN:
            raise RuntimeError("Emulation did not return normally")
        return self.uc.reg_read(UC_ARM_REG_R0)

    def fk(self, joints):
        self.uc.mem_write(INPUT, struct.pack("<6f", *joints, 0.0))
        self.uc.mem_write(OUTPUT, b"\x00" * 64)
        ret = self.call("robot_kinematics_forward", (INPUT, OUTPUT))
        values = struct.unpack("<16f", self.uc.mem_read(OUTPUT, 64))
        return ret, [list(values[i:i + 4]) for i in range(0, 16, 4)]

    def feedback_math(self):
        rows = []
        for i in range(5):
            base = (self.symbols["g_robot"]["address"] + self.robot_layout["members"]["joints"]
                    + i * self.joint_layout["size"])
            raw = bytes(self.uc.mem_read(base, self.joint_layout["size"]))
            jm = self.joint_layout["members"]
            direction = struct.unpack_from("<I", raw, jm["postive_direction"])[0]
            ratio = struct.unpack_from("<f", raw, jm["reduction_ratio"])[0]
            initial_address = self.symbols["g_joints_init"]["address"] + i * self.joint_layout["size"]
            initial = struct.unpack("<f", self.uc.mem_read(initial_address + jm["current_angle"], 4))[0]
            for sign in (0, 1):
                magnitude = 1000000
                packet = bytes([0x36, sign]) + magnitude.to_bytes(4, "big") + b"\x6b"
                self.uc.mem_write(INPUT, packet)
                ret = self.call("robot_update_current_angle_from_data", (i, i + 1, INPUT, 7))
                actual = struct.unpack("<f", self.uc.mem_read(base + jm["current_angle"], 4))[0]
                encoder = magnitude * (-1 if sign else 1) * (-1 if direction else 1)
                expected = (initial + encoder * 360 / 65536 / ratio) % 360
                rows.append({"joint": i + 1, "synthetic_encoder_sign": sign, "ret": ret,
                             "binary_deg": actual, "formula_deg": expected,
                             "abs_error_deg": abs(actual - expected)})
        return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, default=ROOT / "Objects" / "Robot_FreeRTOS.axf")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = {"execution": "PC_Unicorn_ARM_emulation_only", "real_hardware_operated": False,
              "physical_model_validated": False,
              "elf_sha256": hashlib.sha256(args.elf.read_bytes()).hexdigest()}
    exit_code = 0
    try:
        math = BinaryMath(args.elf)
        fixtures = {
            "home": [90, 90, 270, 0, 90],
            "left": [84.99, 120.17, 310.87, 345.54, 79.74],
            "front": [90, 119.34, 309.03, 360, 80.42],
            "right": [110.01, 120.17, 310.88, 14.46, 79.74],
        }
        report["poses"] = {}
        for name, joints in fixtures.items():
            ret, compiled = math.fk(joints)
            reference = firmware_fk(joints)
            report["poses"][name] = {
                "joint_deg": joints, "return_code": ret, "compiled_fk": compiled,
                "rotation_element_max_error": max(abs(compiled[i][j] - reference[i][j])
                    for i in range(3) for j in range(3)),
                "position_max_error_mm": max(abs(compiled[i][3] - reference[i][3]) for i in range(3)),
            }
            if ret != 0:
                raise RuntimeError("Compiled FK returned nonzero")
        report["compiled_relative_rotation_deg"] = {
            a + "_to_" + b: rotation_angle(report["poses"][a]["compiled_fk"], report["poses"][b]["compiled_fk"])
            for a, b in (("left", "front"), ("front", "right"), ("left", "right"))
        }
        report["feedback_conversion_fixtures"] = math.feedback_math()
    except Exception as error:
        report["error"] = str(error)
        print("Offline emulation failed:", error, flush=True)
        exit_code = 1
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    print("Report:", args.output, flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
