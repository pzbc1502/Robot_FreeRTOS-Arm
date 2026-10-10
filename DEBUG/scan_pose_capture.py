"""Operator-supervised CAPTURE_CTRL sampling, never a laser/vision demo."""

import argparse
import datetime as dt
import json
import math
from pathlib import Path
import queue
import re
import sys
import threading
import time

import serial

import formal_workflow_demo as protocol
from repeat_position_error_test import ArmReader, circular_diff_deg


EXPECTED = {1: (70.0, 330.0, 86.0), 2: (90.0, 0.0, 83.0), 3: (115.0, 35.0, 85.0)}
HEADER = re.compile(r"read_all ret=(-?\d+) elapsed=(\d+) ms missing=0x([\da-fA-F]+)")
JOINT = re.compile(r"^J([1-5]) angle=(-?\d+(?:\.\d+)?)$")


def parse_distances(line):
    fields = line.split()
    if len(fields) != 4 or fields[0].lower() != "distance":
        raise ValueError("Enter: distance d1 d2 d3 (three newly measured distances in mm)")
    values = [int(item) for item in fields[1:]]
    if any(value < 100 or value > 65535 for value in values):
        raise ValueError("Unsafe/out-of-range measurement: all three must be >=100 mm; abort")
    return values


def verify_samples(point_id, first, second):
    if any(len(sample) != 5 or not all(math.isfinite(v) for v in sample) for sample in (first, second)):
        raise ValueError("Two complete finite J1-J5 snapshots are required")
    if any(abs(circular_diff_deg(a, b)) > 0.5 for a, b in zip(first, second)):
        raise ValueError("Feedback changed >0.5 deg during static hold; sample rejected")
    for index, expected in zip((0, 3, 4), EXPECTED[point_id]):
        if abs(circular_diff_deg(second[index], expected)) > 3.0:
            raise ValueError("Actual J1/J4/J5 does not match current formal capture recipe")


def verify_next_view_feedback(feedback):
    if len(feedback) != 5 or not all(math.isfinite(v) for v in feedback):
        raise ValueError("Complete finite feedback is required before the next view")
    # read_all also updates firmware current_angle; do not hide a wrapped J4 here.
    for index, lower, upper in ((3, -90.0, 90.0), (4, 0.0, 92.0)):
        if not lower <= feedback[index] <= upper:
            raise RuntimeError(
                f"Unsafe follow-up: J{index + 1} read_all={feedback[index]:.2f} "
                f"outside [{lower:.0f},{upper:.0f}]. Firmware angle representation "
                "must be fixed before sending another capture command."
            )


class Logger:
    def __init__(self, folder):
        folder.mkdir(parents=True, exist_ok=True)
        self.stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = folder / f"scan_pose_capture_{self.stamp}.log"
        self.file = self.path.open("x", encoding="utf-8")
        self.lock = threading.Lock()

    def write(self, message):
        line = f"[{dt.datetime.now().isoformat(timespec='milliseconds')}] {message.rstrip()}\n"
        with self.lock:
            self.file.write(line)
            self.file.flush()
            if not re.search(r"\[ARM-RX\]\s*\[\d+\]\s*<", message):
                print(line, end="", flush=True)


class Session:
    def __init__(self, args, log):
        self.args, self.log = args, log
        self.stop = threading.Event()
        self.cancel = threading.Event()
        self.inputs = queue.Queue()
        self.ports, self.workers = [], []
        self.arm = self.writer = self.reader = self.arm_reader = None
        self.sequences = protocol.SequenceCounter()
        self.started = False
        self.aborted = False

    def open(self, name):
        port = serial.Serial(port=None, baudrate=self.args.baud, timeout=0.05, write_timeout=1)
        port.dtr = False
        port.rts = False
        port.port = name
        port.open()
        self.ports.append(port)
        return port

    def start(self):
        self.arm = self.open(self.args.arm_port)
        jetson = self.open(self.args.jetson_port)
        self.writer = protocol.PortWriter(jetson)
        self.reader = protocol.StatusReader(jetson, self.log, self.stop)
        self.arm_reader = ArmReader(self.arm, self.log, self.stop)
        heart = protocol.HeartbeatSender(self.writer, self.sequences, self.stop, 0.2)
        self.workers = [self.reader, self.arm_reader, heart]
        for worker in self.workers:
            worker.start()
        threading.Thread(target=self._stdin, daemon=True).start()
        time.sleep(0.5)

    def _stdin(self):
        for line in sys.stdin:
            if line.strip().lower() == "stop":
                self.cancel.set()
            self.inputs.put(line.strip())
        self.cancel.set()

    def healthy(self):
        if self.cancel.is_set():
            raise RuntimeError("Operator STOP or input closed")
        if any(not worker.thread.is_alive() for worker in self.workers):
            raise RuntimeError("Serial reader/heartbeat stopped")
        while True:
            try:
                frame = self.reader.events.get_nowait()
            except queue.Empty:
                break
            self.reader.pending.append(frame)
        for frame in self.reader.pending:
            if frame.msg_type == protocol.MSG_ERROR or (
                frame.msg_type == protocol.MSG_STATUS and len(frame.payload) == 3 and frame.payload[2]
            ):
                raise RuntimeError("Firmware reported a fault/rejection")

    def pause(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.healthy()
            time.sleep(0.05)

    def arm_command(self, command):
        if command not in ("laser_off", "read_all", "motion_abort"):
            raise ValueError("Only non-starting ARM commands are permitted")
        protocol.send_arm_command(self.arm, self.log, command)

    def wait(self, event, value, seq, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.healthy()
            try:
                frame = self.reader.wait_status(event, value, seq, timeout=0.2)
            except TimeoutError:
                continue
            if frame.payload[2] != 0:
                raise RuntimeError(f"Status error 0x{frame.payload[2]:02X}")
            return frame
        raise TimeoutError(f"State timeout: event=0x{event:02X} value={value} seq={seq}")

    def control(self, msg_type, payload, label):
        allowed = (
            (msg_type == protocol.MSG_WORKFLOW_CTRL and payload in (b"\x01", b"\x03")) or
            (msg_type == protocol.MSG_CAPTURE_CTRL and
             payload in (b"\x02\x00", b"\x01\x01", b"\x01\x02", b"\x01\x03")) or
            (msg_type == protocol.MSG_TARGET_CTRL and payload == b"\x00")
        )
        if not allowed:
            raise ValueError("Command is outside the non-laser capture workflow")
        self.healthy()
        seq = self.sequences.next()
        frame = protocol.build_frame(msg_type, seq, payload)
        self.log.write(f"[JETSON-TX] {label} seq={seq}: {protocol.hex_bytes(frame)}")
        if msg_type == protocol.MSG_WORKFLOW_CTRL and payload == b"\x01":
            self.started = True
        self.writer.send(frame)
        ack = self.wait(protocol.EVENT_COMMAND_ACK, None, seq, 5)
        if ack.payload[1] != 1:
            raise RuntimeError(f"{label} was rejected")
        return seq

    def feedback(self):
        self.arm_reader.drain()
        self.arm_command("read_all")
        deadline = time.monotonic() + 8
        values = {}
        valid_header = False
        while time.monotonic() < deadline:
            self.healthy()
            try:
                line = self.arm_reader.lines.get(timeout=0.1)
            except queue.Empty:
                continue
            header = HEADER.search(line)
            if header:
                if valid_header or int(header[1]) != 0 or int(header[3], 16) != 0:
                    raise RuntimeError("Failed or ambiguous read_all snapshot")
                valid_header = True
            match = JOINT.fullmatch(line.strip())
            if match and valid_header:
                index, angle = int(match[1]), float(match[2])
                if index in values or not math.isfinite(angle):
                    raise RuntimeError("Invalid read_all joint feedback")
                values[index] = angle
                if len(values) == 5:
                    return [values[i] for i in range(1, 6)]
        raise TimeoutError("Incomplete J1-J5 CAN feedback")

    def distances(self):
        while not self.inputs.empty():
            self.inputs.get_nowait()
            self.log.write("[INPUT-WARN] Input received before measurement readiness was discarded")
        self.log.write("[INPUT_REQUIRED] 公共测距位已到位。请测量3次实际距离(mm)，输入 distance d1 d2 d3；stop 中止。")
        deadline = time.monotonic() + self.args.input_timeout
        while time.monotonic() < deadline:
            self.healthy()
            try:
                line = self.inputs.get(timeout=0.1)
            except queue.Empty:
                continue
            values = parse_distances(line)
            self.log.write(f"[MEASUREMENT] source=operator_actual_measurement distance_mm={values}")
            return values
        raise TimeoutError("No real distance input; abort and hold, not automatic HOME")

    def abort(self):
        if self.aborted:
            return
        self.aborted = True
        if self.writer is not None and self.started:
            try:
                seq = self.sequences.next()
                self.writer.send(protocol.build_frame(protocol.MSG_WORKFLOW_CTRL, seq, b"\x03"))
                response = self.reader.wait_status(protocol.EVENT_WORKFLOW,
                    protocol.WORKFLOW_ABORTED_HOLD, seq, timeout=3)
                if response.payload[2]:
                    raise RuntimeError("ABORT status error")
                self.log.write("[STOP] ABORT_HOLD acknowledged; this is not independent physical stop confirmation")
            except Exception as error:
                self.log.write(f"[STOP-WARN] ABORT not confirmed: {error}; use physical emergency stop if needed")
                try:
                    self.arm_command("motion_abort")
                except Exception:
                    pass
        if self.arm is not None:
            try:
                self.arm_command("laser_off")
            except Exception as error:
                self.log.write(f"[STOP-WARN] laser_off failed: {error}")

    def close(self):
        self.abort()
        self.stop.set()
        for worker in self.workers:
            worker.thread.join(timeout=2)
        for port in self.ports:
            port.close()


def run(session, args, log):
    data = json.loads((Path(__file__).with_name("scan_geometry_input.template.json")).read_text(encoding="utf-8"))
    output = args.debug_dir / f"scan_pose_capture_{log.stamp}.json"
    metadata = {"distance_source": "operator_actual_measurement", "samples": {}, "complete": False}

    def save():
        output.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        output.with_suffix(".metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    session.start()
    session.arm_command("laser_off")
    session.control(protocol.MSG_TARGET_CTRL, b"\x00", "TARGET DISABLE")
    start = session.control(protocol.MSG_WORKFLOW_CTRL, b"\x01", "START: HOME then public measurement position")
    session.wait(protocol.EVENT_WORKFLOW, protocol.WORKFLOW_MEASURE_READY, start, 75)
    session.log.write("[MEASURE_READY] 正式公共测距位到位；保持心跳，尚未发送安全距离或三视图命令。")
    metadata["public_pose_joint_deg"] = session.feedback()
    values = session.distances()
    metadata["distance_samples_mm"] = values
    for value in values:
        seq = session.sequences.next()
        session.writer.send(protocol.build_frame(protocol.MSG_SAFE_DISTANCE, seq,
            protocol.struct.pack("<HB", value, 1)))
        session.log.write(f"[JETSON-TX] actual SAFE_DISTANCE seq={seq} distance={value}mm")
        session.pause(0.2)
    session.wait(protocol.EVENT_WORKFLOW, protocol.WORKFLOW_SAFE_LATCHED, start, 5)
    home = session.control(protocol.MSG_CAPTURE_CTRL, b"\x02\x00", "CAPTURE initial HOME")
    session.wait(protocol.EVENT_CAPTURE_HOME, 1, home, 55)
    last_feedback = None
    for point_id, name in ((1, "left"), (2, "front"), (3, "right")):
        if last_feedback is not None:
            verify_next_view_feedback(last_feedback)
        log.write(f"[MOVING] 准备前往正式 {name} 视图；仅执行此点，等待真实到位。")
        seq = session.control(protocol.MSG_CAPTURE_CTRL, bytes((1, point_id)), f"CAPTURE GOTO {name}")
        session.wait(protocol.EVENT_CAPTURE_POINT, point_id, seq, 60)
        session.pause(0.8)
        first = session.feedback()
        session.pause(0.5)
        second = session.feedback()
        verify_samples(point_id, first, second)
        last_feedback = second
        data["poses"][name] = {"joint_deg": second, "feedback_valid": True}
        metadata["samples"][name] = {"capture_seq": seq, "first_joint_deg": first,
            "second_joint_deg": second, "recorded_at": dt.datetime.now().isoformat(timespec="milliseconds")}
        save()
        log.write(f"[CAPTURED] {name} joint_deg={second}")
        session.pause(2)
    home = session.control(protocol.MSG_CAPTURE_CTRL, b"\x02\x00", "CAPTURE final HOME")
    session.wait(protocol.EVENT_CAPTURE_HOME, 1, home, 55)
    metadata["home_joint_deg"] = session.feedback()
    if any(abs(circular_diff_deg(actual, expected)) > 2.0 for actual, expected in
           zip(metadata["home_joint_deg"], (90.0, 90.0, -90.0, 0.0, 90.0))):
        raise RuntimeError("Final measured HOME does not match the current firmware recipe")
    metadata["complete"] = True
    save()
    log.write(f"[COMPLETE] 三视图角度采集完成并回 HOME: {output.resolve()}")
    log.write("[SCOPE] Camera extrinsic still uncalibrated; this is not a safe/feasible continuous-scan verdict.")


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-port", default="COM3")
    parser.add_argument("--jetson-port", default="COM16")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--input-timeout", type=float, default=180)
    parser.add_argument("--debug-dir", type=Path, default=Path(__file__).resolve().parent / "scan_pose_readback")
    parser.add_argument("--operator-confirmed", action="store_true",
                        help="fixed model, no person in motion range, operator present with emergency stop")
    args = parser.parse_args()
    if not args.operator_confirmed or not math.isfinite(args.input_timeout) or not 1 <= args.input_timeout <= 600:
        parser.error("Operator safety confirmation and input timeout 1..600 seconds are required")
    if args.arm_port.upper() == args.jetson_port.upper():
        parser.error("ARM and Jetson ports must differ")
    log = Logger(args.debug_dir)
    session = Session(args, log)
    try:
        log.write(f"[SESSION] ARM={args.arm_port} JETSON={args.jetson_port} baud={args.baud}; type stop to abort")
        run(session, args, log)
        return 0
    except (Exception, KeyboardInterrupt) as error:
        log.write(f"[ABORT] {error}")
        return 1
    finally:
        session.close()
        log.file.close()


if __name__ == "__main__":
    sys.exit(main())
