"""Record source and existing artifact hashes without claiming a matching build."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def snapshot(root=ROOT):
    def git(*args):
        return subprocess.check_output(['git', '-c', 'core.safecrlf=false', '-C', str(root), *args]).decode('utf-8', errors='replace').strip()
    files = []
    for directory in ('APP', 'Middle', 'USER_BSP', 'src', 'ra_cfg', 'ra_gen', 'ra', 'MQTT', 'RTE', 'script', 'UpperComputer'):
        files.extend(p for p in (root/directory).rglob('*') if p.is_file() and
                     not any(part.startswith(('__pycache__', 'build', 'dist')) for part in p.relative_to(root).parts) and p.suffix in ('.c', '.h', '.s', '.S', '.asm', '.py', '.scat', '.spec', '.txt'))
    files.extend(root/name for name in ('Robot_FreeRTOS.uvprojx', 'configuration.xml', '.gitattributes', '.gitignore', 'rasc_version.txt',
                                    'doc/Jetson↔RA6M5 通信协议.md', 'doc/项目演示.md', 'doc/维护与版本基线.md')
                 if (root/name).is_file())
    artifacts = [root/p for p in ('Objects/Robot_FreeRTOS.axf', 'Objects/Robot_FreeRTOS.hex',
                                  'UpperComputer/dist/RA6M5_Robot_Console.exe') if (root/p).is_file()]
    target = (root/'APP/robot_target.h').read_text(encoding='utf-8')
    tolerance = re.search(r'^#define TARGET_ALIGN_TOL_PX\s+\(([^)]+)\)', target, re.M).group(1)
    return dict(created_utc=datetime.now(timezone.utc).isoformat(), root=str(root.resolve()),
                git_head=git('rev-parse', 'HEAD'), worktree_status=git('status', '--porcelain=v1'),
                contract='CONTINUOUS_SCAN_20261013', target_align_tol_px=tolerance,
                source_sha256={str(p.relative_to(root)): sha256(p) for p in sorted(set(files))},
                existing_artifacts={str(p.relative_to(root)): sha256(p) for p in artifacts},
                artifact_source_match='NOT_VERIFIED', hardware_acceptance='NOT_PERFORMED')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/baseline/snapshot.json')
    args = parser.parse_args()
    result = snapshot()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(args.output)


if __name__ == '__main__':
    main()
