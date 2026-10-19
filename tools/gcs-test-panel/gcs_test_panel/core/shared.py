"""Imports TEST-004's mission-orchestrator-sim package rather than forking it.

Its dialect generation (mc_mavlink_defs.xml -> pymavlink module via mavgen)
and hand-rolled MAVFTP client were verified against the real
mission_orchestrator_node; re-implementing either here would mean
re-verifying them. tools/mission-orchestrator-sim is a sibling directory in
this checkout, so it is put on sys.path once, here.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

_SIM_ROOT = Path(
    os.environ.get(
        "MISSION_ORCHESTRATOR_SIM_ROOT",
        Path(__file__).resolve().parents[3] / "mission-orchestrator-sim",
    )
)
if not (_SIM_ROOT / "mission_orchestrator_sim").is_dir():
    raise ImportError(
        f"tools/mission-orchestrator-sim not found at {_SIM_ROOT} -- set "
        "MISSION_ORCHESTRATOR_SIM_ROOT if running outside a swarm-drones checkout"
    )
if str(_SIM_ROOT) not in sys.path:
    sys.path.insert(0, str(_SIM_ROOT))

from mission_orchestrator_sim import dialect  # noqa: E402
from mission_orchestrator_sim.transport import (  # noqa: E402
    ChannelCClient,
    DirectoryEntry,
    MavftpError,
)

_loaded = None


def load_dialect(force_regen: bool = False):
    """Generate/import the mc_mavlink_defs dialect and make it pymavlink's
    active one. Must run before any link is constructed (mavfile.__init__
    binds the dialect module at construction time)."""
    global _loaded
    if _loaded is None or force_regen:
        _loaded = dialect.load_dialect(force_regen=force_regen)
        dialect.install_dialect(_loaded)
    return _loaded


__all__ = ["ChannelCClient", "DirectoryEntry", "MavftpError", "load_dialect"]
