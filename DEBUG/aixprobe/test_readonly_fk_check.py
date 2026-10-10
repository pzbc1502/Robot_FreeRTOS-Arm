"""Offline tests only. No network connection, serial port, or real MCU access."""

import struct
import unittest

from readonly_fk_check import MCP, ROOT, historical_comparison
from emulate_fk_binary import BinaryMath
from scan_geometry_preflight import firmware_fk


class ReadOnlyGuardTests(unittest.TestCase):
    def setUp(self):
        self.client = object.__new__(MCP)
        self.client.request = lambda *a, **k: {"content": [{"type": "text", "text": "{}"}]}

    def test_target_modifying_tools_are_rejected(self):
        for command in ("halt", "resume", "reset", "step", "memory_write", "flash_write", "breakpoint_set"):
            with self.subTest(command=command), self.assertRaises(ValueError):
                self.client.call(command)

    def test_raw_commands_cannot_escape_read_only_scope(self):
        for command in ("halt", "reset run", "mww 0x20000000 1", "poll; halt", "read_memory 0x0 32 129"):
            with self.subTest(command=command), self.assertRaises(ValueError):
                self.client.call("raw_command", {"command": command})

    def test_small_read_commands_are_allowed(self):
        for command in ("poll", "targets", "read_memory 0x20000000 32 128"):
            self.assertEqual(self.client.call("raw_command", {"command": command}), {})

    def test_word_reads_preserve_little_endian_order(self):
        self.client.call = lambda *a, **k: {"response": "0x410fd214 0x00000001"}
        self.assertEqual(self.client.read(0, 8), struct.pack("<II", 0x410FD214, 1))

    def test_truncated_memory_is_rejected(self):
        self.client.call = lambda *a, **k: {"response": "0x0"}
        with self.assertRaises(RuntimeError):
            self.client.read(0, 8)

    def test_non_aligned_reads_are_rejected(self):
        with self.assertRaises(ValueError):
            self.client.read(1, 4)

    def test_historical_sign_hypotheses_are_not_a_motor_fix(self):
        result = historical_comparison()
        self.assertTrue(result["hypotheses_not_validated"])
        self.assertTrue(result["not_motor_commands"])
        values = result["variants_deg"]
        self.assertGreater(values["current_fk"]["left_to_right"], 50)
        self.assertLess(values["reflect_j1_about_home"]["left_to_right"], 7)
        self.assertEqual(values["reflect_j1_about_home"], values["reflect_j4_about_home"])


class CompiledMathTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.math = BinaryMath(ROOT / "Objects" / "Robot_FreeRTOS.axf")

    def test_compiled_fk_matches_reference_at_four_recorded_poses(self):
        for joints in ([90, 90, 270, 0, 90], [84.99, 120.17, 310.87, 345.54, 79.74],
                       [90, 119.34, 309.03, 360, 80.42], [110.01, 120.17, 310.88, 14.46, 79.74]):
            with self.subTest(joints=joints):
                ret, actual = self.math.fk(joints)
                expected = firmware_fk(joints)
                self.assertEqual(ret, 0)
                self.assertLess(max(abs(actual[i][j] - expected[i][j]) for i in range(3) for j in range(3)), 1e-5)
                self.assertLess(max(abs(actual[i][3] - expected[i][3]) for i in range(3)), 1e-3)

    def test_zero_and_full_turn_have_equivalent_orientation(self):
        _, zero = self.math.fk([90, 119.34, 309.03, 0, 80.42])
        _, turn = self.math.fk([90, 119.34, 309.03, 360, 80.42])
        self.assertLess(max(abs(zero[i][j] - turn[i][j]) for i in range(3) for j in range(3)), 1e-5)

    def test_compiled_encoder_conversion_matches_formula(self):
        results = self.math.feedback_math()
        self.assertEqual(len(results), 10)
        for sample in results:
            self.assertEqual(sample["ret"], 0)
            self.assertLess(sample["abs_error_deg"], 1e-3)

    def test_emulator_only_allows_math_entrypoints(self):
        with self.assertRaises(ValueError):
            self.math.call("robot_joint_rotate_to", ())


if __name__ == "__main__":
    unittest.main()
