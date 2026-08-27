"""Loads the mc_mavlink_defs MAVLink dialect used on Channel C.

Generates the pymavlink dialect module from ws/src/gcs_comms/mc_mavlink_defs.xml
with mavgen (TEST-004 AC: "the custom dialect module mavgen generates from
mc_mavlink_defs.xml") and installs it as pymavlink.mavutil's active dialect,
so every mavutil/mavftp helper this tool uses decodes MC_MISSION_PLAN_ACTIVATE
(50104) and MC_MISSION_PLAN_ACK (50105) exactly like the common.xml messages —
no hand-rolled raw-frame decoding of the kind mission_orchestrator_node.cpp
needs on the C++ side (its own mc_mavlink_defs.xml header comment explains why
it can't just include a generated header there). Generation happens into a
local cache directory, not into pymavlink's own installed package tree
(pymavlink.mavutil.set_dialect()'s own auto-generate fallback writes there,
which would require write access to site-packages and pollute a shared
install) — this is a standalone tool, so it stays fully self-contained.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import os
import shutil
import sys
import tempfile
from pathlib import Path

_TOOL_ROOT = Path(__file__).resolve().parent.parent
_DEFAULT_XML = _TOOL_ROOT.parent.parent / "ws" / "src" / "gcs_comms" / "mc_mavlink_defs.xml"


def _xml_path() -> Path:
    override = os.environ.get("MC_MAVLINK_DEFS_XML")
    return Path(override) if override else _DEFAULT_XML


def _cache_dir() -> Path:
    override = os.environ.get("MISSION_ORCHESTRATOR_SIM_CACHE")
    path = Path(override) if override else _TOOL_ROOT / "_generated"
    path.mkdir(parents=True, exist_ok=True)
    return path


def _pymavlink_v20_dir() -> Path:
    import pymavlink

    return Path(pymavlink.__file__).resolve().parent / "dialects" / "v20"


def _generate(xml_path: Path, cache_dir: Path, module_name: str) -> Path:
    from pymavlink.generator import mavgen

    with tempfile.TemporaryDirectory() as td:
        td_path = Path(td)
        # mavgen resolves <include>common.xml</include> relative to
        # mc_mavlink_defs.xml's own directory (ws/src/gcs_comms/, which does
        # not carry a copy of common.xml) — stage pymavlink's own bundled
        # v2.0 dialect XMLs (common/standard/minimal) alongside a copy of our
        # file for this one-off generation instead.
        for xml in _pymavlink_v20_dir().glob("*.xml"):
            shutil.copy(xml, td_path / xml.name)
        local_defs = td_path / "mc_mavlink_defs.xml"
        shutil.copy(xml_path, local_defs)

        opts = argparse.Namespace(
            wire_protocol="2.0",
            language="Python3",
            output=str(td_path / module_name),
            validate=False,
            strict_units=False,
            error_limit=200,
        )
        mavgen.mavgen(opts, [str(local_defs)])

        generated = td_path / f"{module_name}.py"
        out_path = cache_dir / f"{module_name}.py"
        shutil.copy(generated, out_path)
        return out_path


def load_dialect(force_regen: bool = False):
    """Generate (if needed) and import the mc_mavlink_defs dialect module.

    Cached by a hash of mc_mavlink_defs.xml's own contents, so an edit to
    that file (e.g. a future message added to the 50100-50199 range)
    regenerates automatically rather than silently reusing a stale module.
    """
    xml_path = _xml_path()
    if not xml_path.is_file():
        raise FileNotFoundError(
            f"mc_mavlink_defs.xml not found at {xml_path} — set MC_MAVLINK_DEFS_XML "
            "to point at ws/src/gcs_comms/mc_mavlink_defs.xml if running this tool "
            "outside a checkout of the swarm-drones monorepo"
        )

    digest = hashlib.sha256(xml_path.read_bytes()).hexdigest()[:16]
    module_name = f"mc_mavlink_defs_{digest}"
    cache_dir = _cache_dir()
    gen_py = cache_dir / f"{module_name}.py"

    if force_regen or not gen_py.is_file():
        gen_py = _generate(xml_path, cache_dir, module_name)

    spec = importlib.util.spec_from_file_location(module_name, gen_py)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def install_dialect(module) -> None:
    """Point pymavlink.mavutil's active dialect at `module`.

    mavfile.__init__ builds self.mav = mavlink.MAVLink(...) using mavutil's
    own module-level `mavlink` name, so setting that attribute here is
    enough for every mavutil.mavfile/mavudp subclass constructed afterwards
    to pick up the custom dialect — including MC_MISSION_PLAN_ACTIVATE/ACK,
    which mavutil.set_dialect()'s normal pymavlink.dialects.v20.* import path
    has never heard of.
    """
    import pymavlink.mavutil as mavutil

    mavutil.mavlink = module
    mavutil.current_dialect = module.__name__
