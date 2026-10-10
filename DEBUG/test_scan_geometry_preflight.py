"""Synthetic offline tests; no hardware or compiled C binary validation."""

from contextlib import redirect_stderr, redirect_stdout
import copy
import io
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

import scan_geometry_preflight as geometry


HOME_JOINTS = [90.0, 90.0, -90.0, 0.0, 90.0]
HOME_MATRIX = [
    [0.0, -1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0, -47.63],
    [1.0, 0.0, 0.0, 15.5],
    [0.0, 0.0, 0.0, 1.0],
]
# Independent constants derived from the chain's closed-form basis and position,
# not from firmware_fk output. C-source/runtime checking belongs to the parent.
ASYMMETRIC_CASES = (
    ([30.0, 120.0, -45.0, -20.0, 75.0], [
        [-0.031203732841921947, -0.18374088429429047, -0.982479320136298, -167.80035557922713],
        [0.08420014027506077, 0.9789807261240498, -0.18576079848904709, -96.87958046378168],
        [0.9959601716104867, -0.08852132690137696, -0.015076844803522588, 7.319316924787756],
        [0.0, 0.0, 0.0, 1.0],
    ]),
    ([10.0, 110.0, -70.0, 30.0, 45.0], [
        [-0.5416752204197017, -0.6427876096865393, -0.541675220419702, -114.27121104266453],
        [-0.4545194776720436, 0.7660444431189781, -0.4545194776720436, -20.149097625052836],
        [0.7071067811865476, 0.0, -0.7071067811865475, 3.438524157181689],
        [0.0, 0.0, 0.0, 1.0],
    ]),
)


def synthetic_input():
    return {
        "schema_version": 1,
        "camera": {
            "mount": "after_j5",
            "optical_frame": "rgb",
            "calibration_id": "synthetic_test_only",
            "calibrated": True,
            "T_tool_camera": [
                [0.0, -1.0, 0.0, 10.0],
                [1.0, 0.0, 0.0, 20.0],
                [0.0, 0.0, 1.0, 30.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            "intrinsics": dict(geometry.INTRINSICS),
        },
        "poses": {name: {"joint_deg": list(HOME_JOINTS), "feedback_valid": True}
                  for name in ("left", "front", "right")},
    }


class GeometryTests(unittest.TestCase):
    def assertVectorClose(self, actual, expected, delta=1e-9):
        self.assertEqual(len(actual), len(expected))
        for value, reference in zip(actual, expected):
            self.assertAlmostEqual(value, reference, delta=delta)

    def assertMatrixClose(self, actual, expected):
        self.assertEqual(len(actual), 4)
        for row, reference in zip(actual, expected):
            self.assertVectorClose(row, reference)

    def test_home_absolute_base_degrees_mm_and_fixed_j6(self):
        self.assertMatrixClose(geometry.firmware_fk(HOME_JOINTS), HOME_MATRIX)

    def test_asymmetric_closed_form_golden_matrices(self):
        for joints, expected in ASYMMETRIC_CASES:
            with self.subTest(joints=joints):
                self.assertMatrixClose(geometry.firmware_fk(joints), expected)

    def test_camera_composition_and_opencv_optical_axis(self):
        report = geometry.evaluate(synthetic_input())
        pose = report["poses"]["front"]
        self.assertMatrixClose(pose["T_base_tool"], HOME_MATRIX)
        self.assertMatrixClose(pose["T_base_camera"], [
            [-1.0, 0.0, 0.0, -20.0],
            [0.0, 0.0, -1.0, -77.63],
            [0.0, -1.0, 0.0, 25.5],
            [0.0, 0.0, 0.0, 1.0],
        ])
        self.assertVectorClose(pose["optical_center_base_mm"], [-20.0, -77.63, 25.5])
        self.assertVectorClose(pose["optical_axis_base"], [0.0, -1.0, 0.0])
        self.assertAlmostEqual(report["camera_height_span_mm"], 0.0)

    def test_j5_rotates_camera_and_lever_arm_not_tool_origin(self):
        data = synthetic_input()
        data["poses"]["right"]["joint_deg"][4] = 0.0
        report = geometry.evaluate(data)
        front, right = report["poses"]["front"], report["poses"]["right"]
        self.assertVectorClose([r[3] for r in right["T_base_tool"][:3]],
                               [r[3] for r in front["T_base_tool"][:3]])
        self.assertVectorClose(right["optical_center_base_mm"], [-20.0, -57.63, -14.5])
        self.assertVectorClose(right["optical_axis_base"], [0.0, 0.0, -1.0])
        self.assertNotEqual(front["T_base_tool"], right["T_base_tool"])
        self.assertAlmostEqual(report["camera_height_span_mm"], 40.0)

    def test_front_center_stays_fixed_in_base_and_can_be_behind_left(self):
        data = synthetic_input()
        data["face_center_camera_mm"] = [0.0, 0.0, 200.0]
        data["poses"]["left"]["joint_deg"][0] = 270.0
        report = geometry.evaluate(data)
        self.assertVectorClose(report["face_center_base_mm"], [-20.0, -277.63, 25.5])
        front = report["poses"]["front"]["face"]
        self.assertAlmostEqual(front["optical_axis_angle_deg"], 0.0, delta=1e-9)
        self.assertVectorClose(front["projected_pixel"], [420.925079, 265.676117])
        self.assertTrue(front["visible"])
        left = report["poses"]["left"]["face"]
        self.assertVectorClose(left["point_camera_mm"], [-40.0, 0.0, -355.26])
        self.assertGreater(left["optical_axis_angle_deg"], 90.0)
        self.assertEqual(left["projection_status"], "behind_camera")
        self.assertIsNone(left["projected_pixel"])
        self.assertFalse(left["visible"])

    def test_projection_units_and_opencv_right_down(self):
        expected_pixel = [420.925079 + 409.721191 * 0.05,
                          265.676117 + 409.621887 * 0.1]
        for point in ([10.0, 20.0, 200.0], [100.0, 200.0, 2000.0]):
            with self.subTest(point=point):
                data = synthetic_input()
                data["face_center_camera_mm"] = point
                report = geometry.evaluate(data)
                face = report["poses"]["front"]["face"]
                self.assertVectorClose(face["point_camera_mm"], point)
                self.assertVectorClose(face["projected_pixel"], expected_pixel)
                self.assertAlmostEqual(face["optical_axis_angle_deg"],
                                       math.degrees(math.atan2(math.hypot(10, 20), 200)))
        self.assertVectorClose(geometry.evaluate({**synthetic_input(),
            "face_center_camera_mm": [10.0, 20.0, 200.0]})["face_center_base_mm"],
            [-30.0, -277.63, 5.5])

    def test_outside_image_camera_plane_and_optical_center(self):
        for point, status in (([1000.0, 0.0, 200.0], "projected"),
                              ([10.0, 0.0, 0.0], "on_camera_plane"),
                              ([0.0, 0.0, 0.0], "at_optical_center")):
            with self.subTest(point=point):
                data = synthetic_input()
                data["face_center_camera_mm"] = point
                if point[2] > 0.0:
                    face = geometry.evaluate(data)["poses"]["front"]["face"]
                else:
                    # Transformed anchor diagnostics can have zero Z; a FRONT observation cannot.
                    identity = [[float(i == j) for j in range(4)] for i in range(4)]
                    face = geometry._face_metrics(identity, point)
                self.assertEqual(face["projection_status"], status)
                self.assertFalse(face["visible"])
                if status != "projected":
                    self.assertIsNone(face["projected_pixel"])
                if status == "at_optical_center":
                    self.assertIsNone(face["optical_axis_angle_deg"])

    def test_angle_does_not_overflow_for_finite_large_coordinates(self):
        identity = [[float(i == j) for j in range(4)] for i in range(4)]
        face = geometry._face_metrics(identity, [1.7e308, 1.7e308, 1.7e308])
        self.assertAlmostEqual(face["optical_axis_angle_deg"],
                               math.degrees(math.atan2(math.sqrt(2.0), 1.0)), delta=1e-9)
        self.assertVectorClose(face["projected_pixel"],
                               [geometry.INTRINSICS["fx"] + geometry.INTRINSICS["cx"],
                                geometry.INTRINSICS["fy"] + geometry.INTRINSICS["cy"]])

    def test_report_omits_unknown_face_and_safety_verdicts(self):
        report = geometry.evaluate(synthetic_input())
        self.assertFalse(report["whole_path_evaluated"])
        self.assertNotIn("face_center_base_mm", report)
        self.assertNotIn("face", report["poses"]["front"])
        for key in ("safe", "pass", "safety_distance_mm"):
            self.assertNotIn(key, report)
        json.dumps(report, allow_nan=False)


class ValidationTests(unittest.TestCase):
    def test_front_observation_requires_positive_depth(self):
        for depth in (0.0, -0.0, -1.0, -200.0):
            data = synthetic_input()
            data["face_center_camera_mm"] = [10.0, 0.0, depth]
            with self.subTest(depth=depth), self.assertRaisesRegex(
                    geometry.PreflightError, "FRONT depth Z must be > 0"):
                geometry.evaluate(data)

    def test_required_fields_cannot_be_missing(self):
        paths = [(), ("camera",), ("camera", "intrinsics"), ("poses",)]
        paths += [("poses", name) for name in geometry.POSE_NAMES]
        for path in paths:
            original = synthetic_input()
            target = original
            for key in path:
                target = target[key]
            for key in list(target):
                with self.subTest(path=path, key=key):
                    data = copy.deepcopy(original)
                    target_copy = data
                    for component in path:
                        target_copy = target_copy[component]
                    del target_copy[key]
                    with self.assertRaises(geometry.PreflightError):
                        geometry.evaluate(data)

    def test_null_objects_and_pose_data_reject(self):
        for path in (("camera",), ("poses",), ("camera", "intrinsics"),
                     ("poses", "front")):
            data = synthetic_input()
            target = data
            for key in path[:-1]:
                target = target[key]
            target[path[-1]] = None
            with self.subTest(path=path), self.assertRaises(geometry.PreflightError):
                geometry.evaluate(data)
        with self.assertRaises(geometry.PreflightError):
            geometry.evaluate(None)

    def test_calibration_and_feedback_must_be_explicit(self):
        for value in (False, None, 1, "true"):
            data = synthetic_input()
            data["camera"]["calibrated"] = value
            with self.subTest(calibrated=value), self.assertRaisesRegex(
                    geometry.PreflightError, "calibrated"):
                geometry.evaluate(data)
            data = synthetic_input()
            data["poses"]["front"]["feedback_valid"] = value
            with self.subTest(feedback=value), self.assertRaisesRegex(
                    geometry.PreflightError, "feedback_valid"):
                geometry.evaluate(data)
        for value in ("", "  ", None, True):
            data = synthetic_input()
            data["camera"]["calibration_id"] = value
            with self.subTest(calibration_id=value), self.assertRaisesRegex(
                    geometry.PreflightError, "calibration_id"):
                geometry.evaluate(data)

    def test_numeric_bool_null_nonfinite_and_wrong_lengths_reject(self):
        for value in (True, False, None, "90", float("nan"), float("inf"),
                      -float("inf"), 10 ** 400):
            data = synthetic_input()
            data["poses"]["left"]["joint_deg"][1] = value
            with self.subTest(value=repr(value)), self.assertRaises(geometry.PreflightError):
                geometry.evaluate(data)
        for value in (None, [], [1, 2, 3, 4], [1, 2, 3, 4, 5, 6]):
            with self.subTest(joints=value), self.assertRaises(geometry.PreflightError):
                geometry.firmware_fk(value)
        for value in (None, [0, 0], [True, 0, 200], [0, float("inf"), 200]):
            data = synthetic_input()
            data["face_center_camera_mm"] = value
            with self.subTest(face=value), self.assertRaises(geometry.PreflightError):
                geometry.evaluate(data)

    def test_rigid_transform_shape_finiteness_orthogonality_and_handedness(self):
        for bad in (None, [], [[0] * 3 for _ in range(4)]):
            data = synthetic_input()
            data["camera"]["T_tool_camera"] = bad
            with self.subTest(matrix=bad), self.assertRaises(geometry.PreflightError):
                geometry.evaluate(data)
        for row, col, value in ((0, 0, 0.1), (0, 1, -2.0), (3, 0, 0.01),
                                (3, 3, 0.0), (2, 3, None), (2, 3, True),
                                (2, 3, float("nan")), (2, 3, float("inf")),
                                (2, 2, -1.0)):
            data = synthetic_input()
            data["camera"]["T_tool_camera"][row][col] = value
            with self.subTest(row=row, col=col, value=value), self.assertRaises(
                    geometry.PreflightError):
                geometry.evaluate(data)

    def test_rotation_tolerance(self):
        data = synthetic_input()
        data["camera"]["T_tool_camera"][0][1] -= 1e-8
        geometry.evaluate(data)
        data["camera"]["T_tool_camera"][0][1] -= 1e-4
        with self.assertRaisesRegex(geometry.PreflightError, "orthonormal"):
            geometry.evaluate(data)

    def test_finite_but_overflowing_rotation_rejects(self):
        data = synthetic_input()
        data["camera"]["T_tool_camera"] = [
            [1e308, 1e308, 1e308, 0.0],
            [1e308, -1e308, 1e308, 0.0],
            [1e308, 1e308, -1e308, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        with self.assertRaisesRegex(geometry.PreflightError, "orthonormal"):
            geometry.evaluate(data)

    def test_exact_schema_mount_rgb_and_intrinsics(self):
        for key, bad in (("mount", "before_j5"), ("optical_frame", "depth")):
            data = synthetic_input()
            data["camera"][key] = bad
            with self.subTest(key=key), self.assertRaises(geometry.PreflightError):
                geometry.evaluate(data)
        for key, bad in (("width", True), ("width", 848.0), ("width", 640),
                         ("fx", None), ("fx", float("inf")), ("fy", 0.0),
                         ("cx", 424.0)):
            data = synthetic_input()
            data["camera"]["intrinsics"][key] = bad
            with self.subTest(key=key, bad=bad), self.assertRaises(geometry.PreflightError):
                geometry.evaluate(data)
        for value in (True, 1.0, None, 2):
            data = synthetic_input()
            data["schema_version"] = value
            with self.subTest(schema=value), self.assertRaises(geometry.PreflightError):
                geometry.evaluate(data)
        data = synthetic_input()
        data["AUTO_offset_mm"] = [0, 0, 0]
        with self.assertRaisesRegex(geometry.PreflightError, "unknown"):
            geometry.evaluate(data)


class CliTests(unittest.TestCase):
    def run_cli(self, text, *flags):
        stdout, stderr = io.StringIO(), io.StringIO()
        with patch.object(Path, "read_text", return_value=text) as reader:
            with redirect_stdout(stdout), redirect_stderr(stderr):
                code = geometry.main(["synthetic_input.json", *flags])
            reader.assert_called_once_with(encoding="utf-8-sig")
        return code, stdout.getvalue(), stderr.getvalue()

    def test_positional_json_input_and_utf8_calibration_id(self):
        data = synthetic_input()
        data["camera"]["calibration_id"] = "\u6807\u5b9a-fixture"
        code, output, error = self.run_cli(json.dumps(data, ensure_ascii=False), "--json")
        self.assertEqual(code, 0)
        self.assertEqual(error, "")
        self.assertEqual(json.loads(output)["calibration_id"], data["camera"]["calibration_id"])

    def test_default_human_output_includes_matrices_and_scope(self):
        code, output, error = self.run_cli(json.dumps(synthetic_input()))
        self.assertEqual(code, 0)
        self.assertEqual(error, "")
        for phrase in ("T_base_tool", "T_base_camera", "optical_axis_base",
                       "whole path NOT evaluated", "distortion is not included"):
            self.assertIn(phrase, output)

    def test_invalid_json_and_data_nonzero_without_stdout(self):
        uncalibrated = synthetic_input()
        uncalibrated["camera"]["calibrated"] = False
        invalid_texts = ("{", "null", '{"schema_version":1,"schema_version":1}',
                         json.dumps(uncalibrated),
                         json.dumps(synthetic_input()).replace("90.0", "NaN", 1),
                         json.dumps(synthetic_input()).replace("90.0", "Infinity", 1),
                         json.dumps(synthetic_input()).replace("90.0", "1e999", 1))
        for text in invalid_texts:
            with self.subTest(text=text[:60]):
                code, output, error = self.run_cli(text, "--json")
                self.assertNotEqual(code, 0)
                self.assertEqual(output, "")
                self.assertIn("error:", error)

    def test_file_error_nonzero(self):
        stdout, stderr = io.StringIO(), io.StringIO()
        with patch.object(Path, "read_text", side_effect=OSError("missing input")):
            with redirect_stdout(stdout), redirect_stderr(stderr):
                self.assertEqual(geometry.main(["absent.json"]), 2)
        self.assertEqual(stdout.getvalue(), "")
        self.assertIn("missing input", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
