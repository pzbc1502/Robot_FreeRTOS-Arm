"""Source/model checks, not execution of the compiled RA6M5 FK binary."""

import ast
import math
from pathlib import Path
import re
import unittest

from scan_geometry_preflight import firmware_fk


ROOT = Path(__file__).resolve().parents[1]


def constant_value(expression):
    expression = re.sub(r"(?<=\d)[fF]\b", "", expression.strip())
    node = ast.parse(expression, mode="eval").body

    def number(item):
        if isinstance(item, ast.Constant) and type(item.value) in (int, float):
            return float(item.value)
        if isinstance(item, ast.Name) and item.id == "M_PI":
            return math.pi
        if isinstance(item, ast.UnaryOp) and isinstance(item.op, ast.USub):
            return -number(item.operand)
        if isinstance(item, ast.BinOp) and isinstance(item.op, ast.Div):
            return number(item.left) / number(item.right)
        raise ValueError("Unexpected expression in the firmware DH table")

    return number(node)


def source_dh():
    source = (ROOT / "APP" / "robot.c").read_text(encoding="utf-8")
    match = re.search(r"const\s+float\s+D_H\[6\]\[4\]\s*=\s*\{(.*?)\};", source, re.S)
    if not match:
        raise AssertionError("Current DH declaration not found; review the offline model")
    rows = re.findall(r"\{([^{}]+)\}", match.group(1))
    values = [[constant_value(value) for value in row.split(",")] for row in rows]
    if len(values) != 6 or any(len(row) != 4 for row in values):
        raise AssertionError("Unexpected firmware DH shape")
    return values


def rotation_multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rotation_reference(joints, dh):
    # Independent Rx(alpha) then Rz(theta) composition with parent-frame offsets.
    rotation = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    position = [0.0, 0.0, 0.0]
    for index, (a, alpha, d, _) in enumerate(dh):
        theta = math.radians(joints[index]) if index < 5 else 0.0
        d = d if index < 5 else 0.0
        ca, sa = math.cos(alpha), math.sin(alpha)
        ct, st = math.cos(theta), math.sin(theta)
        offset = [a, -d * sa, d * ca]
        position = [position[i] + sum(rotation[i][k] * offset[k] for k in range(3))
                    for i in range(3)]
        rx = [[1.0, 0.0, 0.0], [0.0, ca, -sa], [0.0, sa, ca]]
        rz = [[ct, -st, 0.0], [st, ct, 0.0], [0.0, 0.0, 1.0]]
        rotation = rotation_multiply(rotation_multiply(rotation, rx), rz)
    return [rotation[i] + [position[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


class FirmwareFkContractTests(unittest.TestCase):
    def assert_matrix_close(self, actual, expected):
        for row in range(4):
            for column in range(4):
                self.assertAlmostEqual(actual[row][column], expected[row][column], places=7)

    def test_source_fk_convention_is_unchanged(self):
        source = (ROOT / "Middle" / "robot_kinematics.c").read_text(encoding="utf-8")
        body = source.split("int robot_kinematics_forward(", 1)[1].split(
            "int robot_kinematics_inverse(", 1)[0]
        body = re.sub(r"/\*.*?\*/|//[^\n]*", "", body, flags=re.S)
        compact = re.sub(r"\s+", "", body)
        for required in (
            "theta_deg=(i==ROBOT_JOINT_6)?0.0f:joint_angle[i];",
            "theta=theta_deg*((float)M_PI/180.0f);",
            "a=D_H[i][0];", "alpha=D_H[i][1];",
            "d=(i==ROBOT_JOINT_6)?0.0f:D_H[i][2];",
            "{ct,-st,0.0f,a}", "{st*ca,ct*ca,-sa,-d*sa}",
            "{st*sa,ct*sa,ca,d*ca}",
            "robot_kinematics_matrix_multiply(total,link,next);",
        ):
            self.assertIn(required, compact, "Firmware FK changed; review the offline port")
        self.assertNotIn("D_H[i][3]", compact)

    def test_home_matches_current_tool_frame(self):
        expected = [[0.0, -1.0, 0.0, 0.0], [0.0, 0.0, -1.0, -47.63],
                    [1.0, 0.0, 0.0, 15.5], [0.0, 0.0, 0.0, 1.0]]
        self.assert_matrix_close(firmware_fk([90, 90, -90, 0, 90]), expected)

    def test_source_dh_matches_offline_math_at_asymmetric_poses(self):
        dh = source_dh()
        # Synthetic math fixtures only, not approved mechanical-arm movement targets.
        for joints in ([90, 90, -90, 0, 90], [70, 110, -65, 330, 86],
                       [115, 120, -55, 35, 85], [43, 137, -23, -41, 52]):
            with self.subTest(joints=joints):
                self.assert_matrix_close(firmware_fk(joints), rotation_reference(joints, dh))


if __name__ == "__main__":
    unittest.main()
