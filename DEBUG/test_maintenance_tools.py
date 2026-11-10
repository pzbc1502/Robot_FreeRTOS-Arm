"""Check offline-runner failure propagation and snapshot source identity."""
import importlib.util
import json
from pathlib import Path
import tempfile
import types
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/f'script/{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    runner = load('check_project')
    with tempfile.TemporaryDirectory() as temp:
        report = Path(temp)/'report.json'
        argv = ['check_project.py', '--only', 'stop_confirmation', '--report', str(report)]
        with patch('sys.argv', argv), patch.object(runner.subprocess, 'run', return_value=types.SimpleNamespace(
                returncode=7, stdout='failure', stderr='')):
            assert runner.main() == 1
            assert json.loads(report.read_text())[0]['exit_code'] == 7
        with patch('sys.argv', ['check_project.py', '--only', 'serial_test']):
            try:
                runner.main()
                raise AssertionError('hardware script entered offline allowlist')
            except SystemExit as error:
                assert error.code == 2
        snap = load('project_snapshot')
        p = Path(temp)/'source.c';p.write_text('one')
        before = snap.sha256(p);p.write_text('two')
        assert before != snap.sha256(p)
    print('maintenance tools: failures propagate, hardware excluded, source hashes change')


if __name__ == '__main__':
    main()
