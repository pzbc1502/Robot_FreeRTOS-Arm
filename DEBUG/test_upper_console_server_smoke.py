from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from UpperComputer.ra6m5_upper_console import server_self_test  # noqa: E402


def test_server_self_test_serves_health_endpoint() -> None:
    server_self_test()


if __name__ == "__main__":
    test_server_self_test_serves_health_endpoint()
    print("upper console server smoke checks passed")
