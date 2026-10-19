import itertools
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gcs_test_panel.core.shared import load_dialect  # noqa: E402

load_dialect()

_ports = itertools.count(24000, 10)


@pytest.fixture
def ports():
    """A fresh block of loopback ports per test: (b_recv, b_send, c_recv, c_send)."""
    base = next(_ports)
    return base + 1, base + 2, base + 3, base + 4


@pytest.fixture
def fake_mc(ports):
    from gcs_test_panel.fake_mc import FakeMC

    b_recv, b_send, c_recv, c_send = ports
    mc = FakeMC(1, b_ports=(b_send, b_recv), c_ports=(c_send, c_recv)).start()
    yield mc
    mc.stop()
