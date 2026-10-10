"""Read-only MCP/ELF inspection. No halt, reset, flash, or motion commands."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import sys
import time
import urllib.request


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE / "python_deps"))
sys.path.insert(0, str(ROOT / "DEBUG"))
from elftools.elf.elffile import ELFFile
from scan_geometry_preflight import firmware_fk


class MCP:
    def __init__(self, url):
        self.url = url
        self.ident = 0
        self.headers = {"Content-Type": "application/json",
                        "Accept": "application/json, text/event-stream"}
        initialized = self.request("initialize", {
            "protocolVersion": "2025-03-26", "capabilities": {},
            "clientInfo": {"name": "readonly-fk-check", "version": "1.0"},
        })
        self.headers["MCP-Protocol-Version"] = initialized["protocolVersion"]
        self.request("notifications/initialized", notification=True)

    def request(self, method, params=None, notification=False):
        body = {"jsonrpc": "2.0", "method": method}
        if not notification:
            self.ident += 1
            body["id"] = self.ident
        if params is not None:
            body["params"] = params
        req = urllib.request.Request(self.url, json.dumps(body).encode(), self.headers)
        with urllib.request.urlopen(req, timeout=15) as response:
            session = response.headers.get("Mcp-Session-Id")
            if session:
                self.headers["Mcp-Session-Id"] = session
            payload = response.read()
        if not payload:
            return None
        try:
            parsed = json.loads(payload)
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise RuntimeError(f"Invalid MCP response for {method}/{params}: {len(payload)} bytes; {error}") from error
        if "error" in parsed:
            raise RuntimeError(parsed["error"])
        return parsed.get("result")

    def call(self, name, arguments=None):
        if name not in ("system_status", "memory_read", "raw_command"):
            raise ValueError("Tool not permitted by this read-only client")
        arguments = arguments or {}
        if name == "raw_command":
            command = arguments.get("command", "")
            memory = re.fullmatch(r"read_memory 0x[0-9a-f]+ 32 (\d+)", command)
            if command not in ("poll", "targets") and not (memory and 1 <= int(memory[1]) <= 128):
                raise ValueError("OpenOCD command not permitted")
        result = self.request("tools/call", {"name": name, "arguments": arguments})
        if result.get("isError"):
            raise RuntimeError(result)
        return json.loads(next(item["text"] for item in result["content"]
                               if item["type"] == "text"))

    def read(self, address, size):
        if address % 4 or size % 4:
            raise ValueError("This inspection uses aligned word reads only")
        output = bytearray()
        # The server's memory_read hex formatter returned invalid/truncated JSON.
        # Use its raw-command tool with bounded, strictly read-only word requests.
        for offset in range(0, size, 512):
            count = min(512, size - offset) // 4
            value = self.call("raw_command", {"command": f"read_memory {hex(address + offset)} 32 {count}"})
            words = value["response"].split()
            if len(words) != count:
                raise RuntimeError(f"Unexpected word count: {len(words)} of {count}")
            output.extend(struct.pack("<" + "I" * count, *(int(word, 0) for word in words)))
        return bytes(output)


def name(die):
    attr = die.attributes.get("DW_AT_name")
    return attr.value.decode(errors="replace") if attr else None


def layout(dwarf, wanted):
    for cu in dwarf.iter_CUs():
        for die in cu.iter_DIEs():
            if die.tag not in ("DW_TAG_structure_type", "DW_TAG_typedef") or name(die) != wanted:
                continue
            if die.tag == "DW_TAG_typedef":
                die = die.get_DIE_from_attribute("DW_AT_type")
            if die.tag != "DW_TAG_structure_type":
                continue
            size = die.attributes.get("DW_AT_byte_size")
            if size is None:
                continue
            members = {}
            for child in die.iter_children():
                if child.tag == "DW_TAG_member":
                    offset = child.attributes["DW_AT_data_member_location"].value
                    if not isinstance(offset, int):
                        raise ValueError("Nonconstant DWARF member location; manual review needed")
                    members[name(child)] = offset
            return {"size": size.value, "members": members}
    raise ValueError("DWARF struct missing: " + wanted)


def u32(raw, offset=0):
    return struct.unpack_from("<I", raw, offset)[0]


def f32(raw, offset=0):
    return struct.unpack_from("<f", raw, offset)[0]


def rotation_angle(a, b):
    trace = sum(a[k][i] * b[k][i] for i in range(3) for k in range(3))
    return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))


def historical_comparison():
    poses = {
        "left": [84.99, 120.17, 310.87, 345.54, 79.74],
        "front": [90.00, 119.34, 309.03, 360.00, 80.42],
        "right": [110.01, 120.17, 310.88, 14.46, 79.74],
    }
    result = {}
    for variant in ("current_fk", "reflect_j1_about_home", "reflect_j4_about_home"):
        matrices = {}
        for key, original in poses.items():
            joints = original.copy()
            if variant == "reflect_j1_about_home":
                joints[0] = 180.0 - joints[0]
            elif variant == "reflect_j4_about_home":
                joints[3] = -joints[3]
            matrices[key] = firmware_fk(joints)
        result[variant] = {
            a + "_to_" + b: rotation_angle(matrices[a], matrices[b])
            for a, b in (("left", "front"), ("front", "right"), ("left", "right"))
        }
    return {"batch": "20261007_221641", "hypotheses_not_validated": True,
            "not_motor_commands": True, "variants_deg": result}


def serial_feedback(port):
    import serial
    evidence = {"port": port, "baud": 115200, "sent_commands": [], "lines": [], "samples": []}
    # Suppress intentional DTR/RTS toggles; no reset or startup commands are sent.
    connection = serial.Serial(port=None, baudrate=115200, timeout=0.1, write_timeout=1)
    connection.dtr = False
    connection.rts = False
    connection.port = port
    with connection:
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            line = connection.readline().decode("utf-8", errors="replace").strip()
            if line:
                evidence["lines"].append(line)
        for _ in range(3):
            connection.write(b"read_all\r\n")
            evidence["sent_commands"].append("read_all")
            sample = {"valid": False, "joint_deg": {}}
            evidence["samples"].append(sample)
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline:
                line = connection.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                evidence["lines"].append(line)
                header = re.search(r"read_all ret=(-?\d+) elapsed=(\d+) ms missing=0x([0-9a-fA-F]+)", line)
                if header:
                    sample.update(ret=int(header[1]), elapsed_ms=int(header[2]),
                                  missing_mask=int(header[3], 16))
                    sample["valid"] = sample["ret"] == 0 and sample["missing_mask"] == 0
                joint = re.fullmatch(r"J([1-5]) angle=(-?[\d.]+)", line)
                if joint and "ret" in sample:
                    sample["joint_deg"][joint[1]] = float(joint[2])
                    if len(sample["joint_deg"]) == 5:
                        break
            sample["valid"] = sample["valid"] and len(sample["joint_deg"]) == 5
            time.sleep(0.25)
    return evidence


def inspect(args, report):
    report["historical_sign_hypotheses"] = historical_comparison()
    with args.elf.open("rb") as stream:
        elf = ELFFile(stream)
        symbols = {s.name: {"address": s["st_value"], "size": s["st_size"]}
                   for s in elf.get_section_by_name(".symtab").iter_symbols() if s.name}
        dwarf = elf.get_dwarf_info()
        robot_layout, joint_layout = layout(dwarf, "robot"), layout(dwarf, "joint")
        report["elf"] = {"path": str(args.elf), "sha256": hashlib.sha256(args.elf.read_bytes()).hexdigest(),
                         "robot_layout": robot_layout, "joint_layout": joint_layout,
                         "can_layout": layout(dwarf, "CAN_Context_t"),
                         "symbols": {key: symbols[key] for key in (
                             "g_robot", "D_H", "g_joints_init", "g_can_context",
                             "robot_kinematics_forward", "g_visual_servo", "s_target", "s_workflow")}}
        flash_segments = [segment for segment in elf.iter_segments()
                          if segment["p_type"] == "PT_LOAD" and segment["p_filesz"]
                          and 0 <= segment["p_paddr"] < 0x100000
                          and segment["p_paddr"] + segment["p_filesz"] <= 0x100000]
        report["elf"]["flash_segments"] = [
            {"address": segment["p_paddr"], "size": segment["p_filesz"]}
            for segment in flash_segments]
        if args.offline:
            return
        client = MCP(args.url)
        status = client.call("system_status")
        report["mcp_status"] = status
        if not status.get("session_active"):
            raise RuntimeError("No active debug session; establish connect separately")
        report["poll"] = client.call("raw_command", {"command": "poll"})
        report["targets"] = client.call("raw_command", {"command": "targets"})
        report["cpuid"] = hex(u32(client.read(0xE000ED00, 4)))
        dhcsr = u32(client.read(0xE000EDF0, 4))
        report["dhcsr"] = {"hex": hex(dhcsr), "halted": bool(dhcsr & (1 << 17)),
                            "sleeping": bool(dhcsr & (1 << 18))}
        matched = 0
        report["flash_match"] = {"complete": False, "matched_bytes": 0}
        for segment in flash_segments:
            data, address = segment.data(), segment["p_paddr"]
            print("Comparing flash segment", hex(address), len(data), flush=True)
            for offset in range(0, len(data), 512):
                expected = data[offset:offset + 512]
                actual = client.read(address + offset, len(expected))
                if actual != expected:
                    first = next(i for i, (a, b) in enumerate(zip(actual, expected)) if a != b)
                    report["flash_match"].update(matched_bytes=matched,
                        mismatch_address=hex(address + offset + first),
                        expected_byte=expected[first], actual_byte=actual[first])
                    raise RuntimeError("ELF/board mismatch; do not interpret RAM with these symbols")
                matched += len(actual)
        if not matched:
            raise RuntimeError("No flash image segments found")
        report["flash_match"].update(complete=True, matched_bytes=matched)
        print("Flash verified:", matched, "bytes", flush=True)
        raw_dh = client.read(symbols["D_H"]["address"], symbols["D_H"]["size"])
        report["live_D_H"] = [list(struct.unpack_from("<4f", raw_dh, i * 16)) for i in range(6)]
        jm, rm = joint_layout["members"], robot_layout["members"]
        def robot_snapshot():
            raw = client.read(symbols["g_robot"]["address"], symbols["g_robot"]["size"])
            joints = []
            for i in range(6):
                base = rm["joints"] + i * joint_layout["size"]
                joints.append({key: (u32 if key in ("postive_direction", "status") else f32)(raw, base + jm[key])
                               for key in ("current_angle", "postive_direction", "reduction_ratio",
                                           "min_angle", "max_angle", "status", "velocity", "acceleration")})
            return {"timestamp_utc": datetime.now(timezone.utc).isoformat(), "coherent_halted_snapshot": False,
                    "status": u32(raw, rm["status"]), "joints": joints,
                    "T_stored_not_necessarily_measured": [list(struct.unpack_from("<4f", raw, rm["T"] + 16 * i)) for i in range(4)]}
        report["before_serial"] = robot_snapshot()
        state = report["before_serial"]["status"]
        if args.port:
            if report["dhcsr"]["halted"] or state & ((1 << 3) | (1 << 7) | (1 << 8)):
                raise RuntimeError("CPU halted or motion mode active; serial sampling skipped")
            try:
                report["serial"] = serial_feedback(args.port)
            except OSError as error:
                report["serial_error"] = str(error)
        report["after_serial"] = robot_snapshot()
        angles = [j["current_angle"] for j in report["after_serial"]["joints"][:5]]
        report["offline_fk_from_live_angles"] = firmware_fk(angles)
        can_raw = client.read(symbols["g_can_context"]["address"], symbols["g_can_context"]["size"])
        report["can_raw_hex"] = can_raw.hex()
        cm = report["elf"]["can_layout"]["members"]
        report["cached_can_motor_dlc"] = list(can_raw[cm["motor_rx_dlc"]:cm["motor_rx_dlc"] + 6])
        report["cached_can_motor_flags"] = list(can_raw[cm["motor_rx_flag"]:cm["motor_rx_flag"] + 6])
        for key in ("g_visual_servo", "s_target", "s_workflow"):
            report[key + "_raw_hex"] = client.read(symbols[key]["address"], symbols[key]["size"]).hex()
        report["joint_feedback_freshness_verified"] = bool(
            report.get("serial", {}).get("samples") and
            all(s["valid"] for s in report["serial"]["samples"]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, default=ROOT / "Objects" / "Robot_FreeRTOS.axf")
    parser.add_argument("--url", default="http://192.168.1.101:8080/mcp")
    parser.add_argument("--port", help="Optional PC text/log port; sends read_all only")
    parser.add_argument("--offline", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = {"timestamp_utc": datetime.now(timezone.utc).isoformat(), "read_only": True,
              "halt_reset_motion_flash_write_performed": False}
    result = 0
    try:
        inspect(args, report)
    except Exception as error:
        report["error"] = str(error)
        print("Inspection stopped:", error, flush=True)
        result = 1
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    print("Report:", args.output, flush=True)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
