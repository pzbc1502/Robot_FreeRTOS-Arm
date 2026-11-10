"""Run an explicit allowlist of offline checks; never discover hardware scripts."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
CORE = '''debug_command_guards hardware_entry_guards maintenance_tools fault_hook
scan_workflow scan_target_handoff stop_confirmation abort_restart global_soft_reset
local_reset_key fire_key_gpio ik_visual_guards safety_review_guards scan_uart
workflow_uart scan_motion can_angle_feedback can_velocity_ack can_error_diagnostics
joint_angle_mapping scan_fk_contract scan_geometry_preflight visual_servo_static
workflow_state_machine_static target_indicator_static target_disable_reset_static
visual_servo_20cm_tuning kp_static robot_49_static 49_static 412_static'''.split()
PC = '''upper_console_protocol upper_console_qt_smoke upper_console_gui_features
upper_console_formal_workflow scan_console console_log_filter'''.split()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pc', action='store_true', help='Also run simulated Qt checks; requires requirements-pc.txt')
    p.add_argument('--only', nargs='+', choices=CORE + PC)
    p.add_argument('--list', action='store_true')
    p.add_argument('--report', type=Path, default=ROOT / 'artifacts/checks/offline.json')
    args = p.parse_args()
    checks = args.only or CORE + (PC if args.pc else [])
    if args.list:
        print('\n'.join(checks)); return 0
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', QT_QPA_PLATFORM='offscreen')
    results = []
    for name in checks:
        start = time.monotonic()
        try:
            result = subprocess.run([sys.executable, '-B', str(ROOT / f'DEBUG/test_{name}.py')],
                                    cwd=ROOT, env=env, capture_output=True, text=True,
                                    encoding='utf-8', errors='replace', timeout=180)
            code, output = result.returncode, result.stdout + result.stderr
        except subprocess.TimeoutExpired:
            code, output = 124, 'Check exceeded 180 seconds'
        results.append(dict(check=name, exit_code=code, seconds=round(time.monotonic()-start, 2), output=output))
        print(f'{"PASS" if code == 0 else "FAIL"} {name}', flush=True)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'{sum(r["exit_code"] == 0 for r in results)}/{len(results)} passed; report: {args.report}')
    return int(any(r['exit_code'] for r in results))


if __name__ == '__main__':
    raise SystemExit(main())
