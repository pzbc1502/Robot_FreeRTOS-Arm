from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TARGET_H = ROOT / "APP" / "robot_target.h"


def macro_value(source: str, name: str) -> float:
    match = re.search(
        rf"^#define\s+{re.escape(name)}\s+\((-?[0-9]+(?:\.[0-9]+)?)[fu]?\)",
        source,
        flags=re.MULTILINE,
    )
    if match is None:
        raise AssertionError(f"missing numeric macro: {name}")
    return float(match.group(1))


def main() -> None:
    source = TARGET_H.read_text(encoding="utf-8")
    kx = macro_value(source, "TARGET_VS_KX_MM_S_PER_PX")
    kz = macro_value(source, "TARGET_VS_KZ_MM_S_PER_PX")
    coarse_limit = macro_value(source, "TARGET_VS_MAX_SPEED_MM_S")
    fine_limit = macro_value(source, "TARGET_VS_FINE_MAX_SPEED_MM_S")
    timeout_ms = macro_value(source, "TARGET_VS_CMD_TIMEOUT_MS")
    tolerance_px = macro_value(source, "TARGET_ALIGN_TOL_PX")

    # Historical filename; this checks the current software baseline, not calibration.
    assert kx == -0.09 and kz == 0.09
    assert coarse_limit == 1.2 and fine_limit == 0.8
    assert timeout_ms == 300.0 and tolerance_px == 2.0
    assert macro_value(source, "TARGET_ALIGN_STABLE_COUNT") == 3
    assert macro_value(source, "TARGET_CONFIRM_STABLE_COUNT") == 2
    assert macro_value(source, "TARGET_VISION_VALID_MS") == 500
    print("current visual-servo parameter baseline passed; no camera calibration claim")



if __name__ == "__main__":
    main()
