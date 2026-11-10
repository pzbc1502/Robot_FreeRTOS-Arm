"""Run real CLI validation with a serial stub that forbids opening any port."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import types
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent


class ForbiddenSerial:
    def __init__(self, *args, **kwargs):
        raise AssertionError('CLI accessed a port before operator confirmation')


def main():
    fake = types.ModuleType('serial')
    fake.Serial = ForbiddenSerial
    fake.SerialException = OSError
    cases = {'serial_test': ['--port', 'COM_TEST'],
             'repeat_position_error_test': ['--port', 'COM_TEST'],
             'target_state_demo': ['--arm-port', 'COM_A', '--jetson-port', 'COM_B'],
             'shooting_robot_demo': ['--arm-port', 'COM_A', '--jetson-port', 'COM_B'],
             'formal_workflow_demo': ['--arm-port', 'COM_A', '--jetson-port', 'COM_B']}
    with patch.dict(sys.modules, serial=fake):
        for name, arguments in cases.items():
            spec = importlib.util.spec_from_file_location(name, ROOT/f'{name}.py')
            module = importlib.util.module_from_spec(spec)
            with patch.dict(sys.modules, {name: module}):
                spec.loader.exec_module(module)
                for argv in ([name], [name, *arguments]):
                    with patch.object(sys, 'argv', argv), contextlib.redirect_stderr(io.StringIO()):
                        try:
                            module.main()
                            raise AssertionError(f'{name} accepted an unconfirmed invocation')
                        except SystemExit as error:
                            assert error.code == 2
                if hasattr(module, 'parse_args'):
                    with patch.object(sys, 'argv', [name, *arguments, '--operator-confirmed']):
                        assert module.parse_args().operator_confirmed
    print('five hardware tools reject missing ports/confirmation before serial access')


if __name__ == '__main__':
    main()
