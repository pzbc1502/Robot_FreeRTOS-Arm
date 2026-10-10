"""Offline tests only: importing this module never opens a serial port."""

from pathlib import Path
import queue
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import scan_pose_capture as capture


class CaptureChecks(unittest.TestCase):
    def test_only_actual_distance_command_is_accepted(self):
        self.assertEqual(capture.parse_distances("distance 201 202 200"), [201, 202, 200])
        for line in ("", "distance 200", "distance 200 200 99", "distance 0 200 200",
                     "distance 200 200 65536", "distance 200 200 200.1", "auto 200 200 200"):
            with self.subTest(line=line), self.assertRaises(ValueError):
                capture.parse_distances(line)

    def test_normalized_angles_and_complete_static_samples(self):
        capture.verify_samples(1, [70, 100, 300, -30, 86], [70.1, 100, 300, 330, 86])
        for first, second in (
            ([70, 100, 300, 330, 86], [72, 100, 300, 330, 86]),
            ([90, 100, 300, 330, 86], [90, 100, 300, 330, 86]),
            ([70, 100, 300, 330], [70, 100, 300, 330, 86]),
            ([70, 100, 300, 330, float("nan")], [70, 100, 300, 330, 86]),
        ):
            with self.subTest(first=first), self.assertRaises(ValueError):
                capture.verify_samples(1, first, second)

    def test_no_laser_enable_vision_or_arbitrary_arm_commands(self):
        session = capture.Session(None, Mock())
        for command in ("soft_reset", "auto 0 -130 -15", "laser_on", "target_enable"):
            with self.subTest(command=command), self.assertRaises(ValueError):
                session.arm_command(command)
        for kind, payload in ((capture.protocol.MSG_TARGET_CTRL, b"\x01"),
                              (capture.protocol.MSG_VISION_ERROR, b"\0" * 5),
                              (capture.protocol.MSG_CAPTURE_CTRL, b"\x03\x01")):
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                session.control(kind, payload, "prohibited")

    def test_wrapped_bounded_joint_blocks_followup_commands(self):
        capture.verify_next_view_feedback([70, 119.34, 309.03, -30, 86.05])
        for angles in ([70, 119.34, 309.03, 330, 86.05],
                       [90, 119.34, 309.03, 360, 83],
                       [90, 119.34, 309.03, 0, 93]):
            with self.subTest(angles=angles), self.assertRaisesRegex(RuntimeError, "Unsafe follow-up"):
                capture.verify_next_view_feedback(angles)

    def test_real_left_readback_is_saved_but_front_is_not_sent(self):
        session = Mock()
        session.sequences = capture.protocol.SequenceCounter()
        session.control.side_effect = range(10, 14)
        session.distances.return_value = [403, 410, 406]
        left = [70, 119.34, 309.03, 330, 86.05]
        session.feedback.side_effect = [[90, 119.35, 309.04, 360, 80.37], left, left]
        with tempfile.TemporaryDirectory() as folder:
            args = SimpleNamespace(debug_dir=Path(folder))
            log = Mock(stamp="fixture")
            with self.assertRaisesRegex(RuntimeError, "J4 read_all=330"):
                capture.run(session, args, log)
            data = capture.json.loads((Path(folder) / "scan_pose_capture_fixture.json").read_text(encoding="utf-8"))
            metadata = capture.json.loads((Path(folder) / "scan_pose_capture_fixture.metadata.json").read_text(encoding="utf-8"))
        self.assertEqual(data["poses"]["left"]["joint_deg"], left)
        self.assertFalse(data["poses"]["front"]["feedback_valid"])
        self.assertFalse(data["poses"]["right"]["feedback_valid"])
        self.assertFalse(metadata["complete"])
        self.assertEqual([call.args[1] for call in session.control.call_args_list],
                         [b"\x00", b"\x01", b"\x02\x00", b"\x01\x01"])

    def test_asynchronous_fault_is_not_hidden_while_waiting(self):
        session = capture.Session(None, Mock())
        session.reader = SimpleNamespace(events=queue.Queue(), pending=[])
        session.reader.events.put(capture.protocol.Frame(capture.protocol.MSG_ERROR, 0, b"\x09"))
        with self.assertRaisesRegex(RuntimeError, "fault"):
            session.healthy()

    def test_reader_death_and_operator_stop_fail_closed(self):
        session = capture.Session(None, Mock())
        worker = Mock()
        worker.thread.is_alive.return_value = False
        session.workers = [worker]
        with self.assertRaisesRegex(RuntimeError, "stopped"):
            session.healthy()
        session.cancel.set()
        with self.assertRaisesRegex(RuntimeError, "STOP"):
            session.healthy()

    def test_capture_sequence_and_saved_input_remain_uncalibrated(self):
        # Synthetic fixture values are not approved physical movement targets.
        session = Mock()
        session.sequences = capture.protocol.SequenceCounter()
        session.control.side_effect = range(10, 17)
        session.distances.return_value = [201, 202, 200]
        home = [90, 90, 270, 360, 90]
        samples = [home]
        for point_id in (1, 2, 3):
            j1, j4, j5 = capture.EXPECTED[point_id]
            if j4 > 180:
                j4 -= 360
            samples.extend([[j1, 100, 300, j4, j5]] * 2)
        samples.append(home)
        session.feedback.side_effect = samples
        with tempfile.TemporaryDirectory() as folder:
            args = SimpleNamespace(debug_dir=Path(folder))
            log = Mock(stamp="fixture")
            capture.run(session, args, log)
            data = capture.json.loads((Path(folder) / "scan_pose_capture_fixture.json").read_text(encoding="utf-8"))
            metadata = capture.json.loads((Path(folder) / "scan_pose_capture_fixture.metadata.json").read_text(encoding="utf-8"))
        self.assertFalse(data["camera"]["calibrated"])
        self.assertIsNone(data["camera"]["T_tool_camera"])
        self.assertTrue(metadata["complete"])
        self.assertTrue(all(value["feedback_valid"] for value in data["poses"].values()))
        self.assertEqual([call.args[1] for call in session.control.call_args_list],
                         [b"\x00", b"\x01", b"\x02\x00", b"\x01\x01", b"\x01\x02", b"\x01\x03", b"\x02\x00"])


if __name__ == "__main__":
    unittest.main()
