"""Offline anchor geometry in the firmware's absolute base frame.

Joint inputs are physical current_angle degrees after motor conversion. Camera
extrinsics map RGB optical coordinates (right, down, forward) into the existing
FK tool frame. Python uses binary64; this is not a compiled C runtime crosscheck.
"""

import argparse
import json
import math
from pathlib import Path
import sys


# APP/robot.c D_H: a [mm], alpha [rad], d [mm], unused angle-offset column.
DH_ROWS = (
    (0.0, 0.0, 0.0, math.pi / 2.0),
    (0.0, math.pi / 2.0, 0.0, math.pi / 2.0),
    (200.0, math.pi, 0.0, -math.pi / 2.0),
    (47.63, -math.pi / 2.0, -184.5, 0.0),
    (0.0, math.pi / 2.0, 0.0, math.pi / 2.0),
    (0.0, math.pi / 2.0, 75.0, 0.0),
)
INTRINSICS = {
    "width": 848, "height": 530,
    "fx": 409.721191, "fy": 409.621887,
    "cx": 420.925079, "cy": 265.676117,
}
POSE_NAMES = ("left", "front", "right")
ROTATION_TOLERANCE = 1e-6
SCOPE_NOTES = (
    "Physical post-conversion joint angles in degrees; lengths in mm.",
    "Absolute FK base coordinates; no AUTO or HOME position offset is applied.",
    "T_tool_camera maps RGB optical coordinates into the FK tool frame.",
    "Face center and FRONT feedback must be paired in the same static hold.",
    "Pinhole projection only; distortion is not included; visibility is point-only.",
    "Only three anchors are evaluated; the whole path is NOT evaluated.",
    "No IK, path planning, collision, clearance, or safety assessment is performed.",
    "Python binary64 math; compiled firmware C runtime equivalence is not verified.",
)


class PreflightError(ValueError):
    """Invalid input or a non-finite geometry calculation."""


def _object(value, path, required, optional=()):
    if not isinstance(value, dict):
        raise PreflightError(f"{path}: expected an object")
    missing = set(required) - value.keys()
    if missing:
        raise PreflightError(f"{path}: missing {', '.join(sorted(missing))}")
    extra = value.keys() - set(required) - set(optional)
    if extra:
        raise PreflightError(f"{path}: unknown field(s): {', '.join(sorted(extra))}")
    return value


def _number(value, path):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PreflightError(f"{path}: expected a finite number, not bool/null/string")
    try:
        result = float(value)
    except (OverflowError, ValueError):
        raise PreflightError(f"{path}: expected a finite number") from None
    if not math.isfinite(result):
        raise PreflightError(f"{path}: expected a finite number")
    return result


def _vector(value, length, path):
    if not isinstance(value, list) or len(value) != length:
        raise PreflightError(f"{path}: expected an array of {length} finite numbers")
    return [_number(item, f"{path}[{i}]") for i, item in enumerate(value)]


def _rigid_transform(value):
    path = "camera.T_tool_camera"
    if not isinstance(value, list) or len(value) != 4:
        raise PreflightError(f"{path}: expected a 4x4 matrix; no extrinsic default exists")
    matrix = [_vector(row, 4, f"{path}[{i}]") for i, row in enumerate(value)]
    if matrix[3] != [0.0, 0.0, 0.0, 1.0]:
        raise PreflightError(f"{path}: homogeneous last row must be [0,0,0,1]")
    for i in range(3):
        for j in range(3):
            dot = sum(matrix[k][i] * matrix[k][j] for k in range(3))
            if not math.isfinite(dot) or abs(dot - float(i == j)) > ROTATION_TOLERANCE:
                raise PreflightError(f"{path}: rotation must be orthonormal (tolerance 1e-6)")
    r = matrix
    determinant = (
        r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
        - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0])
    )
    if not math.isfinite(determinant) or abs(determinant - 1.0) > ROTATION_TOLERANCE:
        raise PreflightError(f"{path}: rotation determinant must be +1 (tolerance 1e-6)")
    return matrix


def validate_input(data):
    root = _object(data, "input", ("schema_version", "camera", "poses"),
                   ("face_center_camera_mm",))
    if type(root["schema_version"]) is not int or root["schema_version"] != 1:
        raise PreflightError("schema_version: expected integer 1")
    camera = _object(root["camera"], "camera", (
        "mount", "optical_frame", "calibration_id", "calibrated",
        "T_tool_camera", "intrinsics",
    ))
    if camera["mount"] != "after_j5":
        raise PreflightError("camera.mount: expected 'after_j5'")
    if camera["optical_frame"] != "rgb":
        raise PreflightError("camera.optical_frame: only 'rgb' is supported")
    if camera["calibrated"] is not True:
        raise PreflightError("camera.calibrated: explicit true is required; calibration is missing")
    calibration_id = camera["calibration_id"]
    if not isinstance(calibration_id, str) or not calibration_id.strip():
        raise PreflightError("camera.calibration_id: expected a nonempty string")
    transform = _rigid_transform(camera["T_tool_camera"])
    intrinsics = _object(camera["intrinsics"], "camera.intrinsics", INTRINSICS)
    for key, expected in INTRINSICS.items():
        actual = _number(intrinsics[key], f"camera.intrinsics.{key}")
        if key in ("width", "height") and type(intrinsics[key]) is not int:
            raise PreflightError(f"camera.intrinsics.{key}: expected integer {expected}")
        if actual != expected:
            raise PreflightError(f"camera.intrinsics.{key}: expected {expected}")
    poses = _object(root["poses"], "poses", POSE_NAMES)
    angles = {}
    for name in POSE_NAMES:
        path = f"poses.{name}"
        pose = _object(poses[name], path, ("joint_deg", "feedback_valid"))
        if pose["feedback_valid"] is not True:
            raise PreflightError(f"{path}.feedback_valid: explicit true is required")
        angles[name] = _vector(pose["joint_deg"], 5, f"{path}.joint_deg")
    face = None
    if "face_center_camera_mm" in root:
        face = _vector(root["face_center_camera_mm"], 3, "face_center_camera_mm")
        if face[2] <= 0.0:
            raise PreflightError("face_center_camera_mm: measured FRONT depth Z must be > 0 mm")
    return calibration_id, transform, angles, face


def _multiply(a, b):
    result = [[sum(a[i][k] * b[k][j] for k in range(4))
               for j in range(4)] for i in range(4)]
    if not all(math.isfinite(x) for row in result for x in row):
        raise PreflightError("geometry: matrix calculation produced a non-finite value")
    return result


def firmware_fk(joint_deg):
    """Port robot_kinematics_forward: degrees in, absolute base/tool matrix out."""
    joints = _vector(joint_deg, 5, "joint_deg")
    total = [[float(i == j) for j in range(4)] for i in range(4)]
    for i, (a, alpha, source_d, _unused_offset) in enumerate(DH_ROWS):
        # J6 contributes its fixed frame rotation only; no motion or 75 mm offset.
        theta = 0.0 if i == 5 else math.radians(joints[i])
        d = 0.0 if i == 5 else source_d
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        link = [
            [ct, -st, 0.0, a],
            [st * ca, ct * ca, -sa, -d * sa],
            [st * sa, ct * sa, ca, d * ca],
            [0.0, 0.0, 0.0, 1.0],
        ]
        total = _multiply(total, link)
    return total


def _transform_point(transform, point):
    result = [sum(transform[i][k] * point[k] for k in range(3))
              + transform[i][3] for i in range(3)]
    if not all(math.isfinite(x) for x in result):
        raise PreflightError("geometry: point calculation produced a non-finite value")
    return result


def _face_metrics(transform, center_base):
    delta = [center_base[i] - transform[i][3] for i in range(3)]
    point = [sum(transform[k][i] * delta[k] for k in range(3)) for i in range(3)]
    if not all(math.isfinite(x) for x in point):
        raise PreflightError("geometry: face coordinate calculation produced a non-finite value")
    x, y, z = point
    at_center = math.hypot(x, y, z) <= 1e-9
    # Scaling preserves the direction even when the unscaled transverse norm overflows.
    scale = max(abs(x), abs(y), abs(z))
    angle = (None if at_center else
             math.degrees(math.atan2(math.hypot(x / scale, y / scale), z / scale)))
    in_front = z > 1e-9 and not at_center
    pixel = None
    if in_front:
        pixel = [INTRINSICS["fx"] * (x / z) + INTRINSICS["cx"],
                 INTRINSICS["fy"] * (y / z) + INTRINSICS["cy"]]
        if not all(math.isfinite(v) for v in pixel):
            raise PreflightError("geometry: pinhole projection produced a non-finite value")
    within = pixel is not None and (0.0 <= pixel[0] < INTRINSICS["width"]
                                   and 0.0 <= pixel[1] < INTRINSICS["height"])
    status = ("at_optical_center" if at_center else
              "on_camera_plane" if abs(z) <= 1e-9 else
              "behind_camera" if z < 0.0 else "projected")
    return {
        "point_camera_mm": point,
        "optical_axis_angle_deg": angle,
        "projected_pixel": pixel,
        "in_front": in_front,
        "within_image": within,
        "visible": in_front and within,
        "projection_status": status,
    }


def evaluate(data):
    calibration_id, extrinsic, angles, face = validate_input(data)
    report = {
        "schema_version": 1,
        "analysis": "anchor_geometry_only",
        "whole_path_evaluated": False,
        "calibration_id": calibration_id,
        "units": {"position": "mm", "joint_angle": "deg", "projection": "pixel"},
        "scope_notes": list(SCOPE_NOTES),
        "poses": {},
    }
    heights = []
    for name in POSE_NAMES:
        tool = firmware_fk(angles[name])
        camera = _multiply(tool, extrinsic)
        center = [camera[i][3] for i in range(3)]
        report["poses"][name] = {
            "T_base_tool": tool,
            "T_base_camera": camera,
            "optical_center_base_mm": center,
            "optical_axis_base": [camera[i][2] for i in range(3)],
        }
        heights.append(center[2])
    span = max(heights) - min(heights)
    if not math.isfinite(span):
        raise PreflightError("geometry: camera height span produced a non-finite value")
    report["camera_height_span_mm"] = span
    if face is not None:
        center_base = _transform_point(report["poses"]["front"]["T_base_camera"], face)
        report["face_center_base_mm"] = center_base
        for pose in report["poses"].values():
            pose["face"] = _face_metrics(pose["T_base_camera"], center_base)
    return report


def _reject_constant(value):
    raise PreflightError(f"JSON: non-finite constant {value} is not allowed")


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise PreflightError(f"JSON: duplicate field {key!r}")
        result[key] = value
    return result


def _human_report(report):
    lines = ["Offline anchor geometry (mm, deg); whole path NOT evaluated.",
             "Calibration: " + json.dumps(report["calibration_id"], ensure_ascii=True)]
    for name, pose in report["poses"].items():
        lines.append(f"{name}:")
        for key in ("T_base_tool", "T_base_camera", "optical_center_base_mm",
                    "optical_axis_base", "face"):
            if key in pose:
                lines.append(f"  {key}: {json.dumps(pose[key], ensure_ascii=True, allow_nan=False)}")
    lines.append(f"camera_height_span_mm: {report['camera_height_span_mm']}")
    if "face_center_base_mm" in report:
        lines.append(f"face_center_base_mm: {report['face_center_base_mm']}")
    lines.extend(report["scope_notes"])
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="UTF-8 JSON input file")
    parser.add_argument("--json", action="store_true", help="emit structured JSON to stdout")
    args = parser.parse_args(argv)
    try:
        data = json.loads(args.input.read_text(encoding="utf-8-sig"),
                          parse_constant=_reject_constant, object_pairs_hook=_unique_object)
        report = evaluate(data)
        output = (json.dumps(report, indent=2, ensure_ascii=True, allow_nan=False)
                  if args.json else _human_report(report))
    except (OSError, UnicodeError, ValueError, OverflowError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    print(output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
