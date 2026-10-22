"""Mission plan model shared by both channels (TEST-007).

One JSON shape serves both paths into the MC:

- Channel C stores it verbatim as a plan file (MAVFTP) -- the format
  mission_orchestrator_node's activate path parses
  (docs/GCS-MAVLINK-API-REFERENCE.md "Mission plan JSON format");
- Channel B encodes the same plan as MC_MISSION_UPLOAD MISSION_ITEM_INTs
  (commands 50200-50205), exactly as gcs_mavlink_node.cpp decodes them.

So a tester edits one plan and can push it either way, and a mismatch between
the two MC-side decoders shows up as a difference in the resulting
/mc/gcs/mission_upload, not as a difference in what the panel sent.
"""

from __future__ import annotations

import ipaddress
import json
from dataclasses import dataclass
from datetime import datetime, timezone

from . import defs


class PlanError(ValueError):
    pass


@dataclass
class MissionItem:
    command: int
    frame: int
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0
    x: int = 0
    y: int = 0
    z: float = 0.0

    def describe(self) -> str:
        return defs.MISSION_ITEM_NAMES.get(self.command, str(self.command))


def _num(obj: dict, key: str, where: str) -> float:
    if key not in obj:
        raise PlanError(f"{where}: missing '{key}'")
    value = obj[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PlanError(f"{where}: '{key}' must be a number, got {value!r}")
    return float(value)


def _list(plan: dict, key: str) -> list:
    value = plan.get(key, [])
    if not isinstance(value, list):
        raise PlanError(f"'{key}' must be a list")
    return value


def validate(plan) -> list[str]:
    """Check `plan` against what mission_orchestrator_node's activate path
    requires (mission_orchestrator_node.cpp's JSON -> MissionUpload
    conversion: `name` mandatory, every listed field present and typed).
    Raises PlanError on the first hard error; returns soft warnings."""
    if not isinstance(plan, dict):
        raise PlanError("plan must be a JSON object")
    if not isinstance(plan.get("name"), str) or not plan["name"]:
        raise PlanError("'name' is required and must be a non-empty string")
    warnings: list[str] = []

    for i, wp in enumerate(_list(plan, "ingress_waypoints")):
        where = f"ingress_waypoints[{i}]"
        lat, lon = _num(wp, "lat", where), _num(wp, "lon", where)
        _num(wp, "alt_m", where)
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise PlanError(f"{where}: lat/lon out of range")
        if lat == 0 and lon == 0:
            warnings.append(f"{where}: lat/lon is 0/0 (Null Island)")
        if "speed_mps" in wp:
            _num(wp, "speed_mps", where)

    for i, fo in enumerate(_list(plan, "formation_offsets")):
        where = f"formation_offsets[{i}]"
        drone_id = _num(fo, "drone_id", where)
        if not (0 <= drone_id <= 255 and drone_id == int(drone_id)):
            raise PlanError(f"{where}: drone_id must be an integer 0-255")
        for key in ("north_m", "east_m", "down_m"):
            _num(fo, key, where)

    polygon = _list(plan, "search_polygon")
    for i, v in enumerate(polygon):
        where = f"search_polygon[{i}]"
        lat, lon = _num(v, "lat", where), _num(v, "lon", where)
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise PlanError(f"{where}: lat/lon out of range")
    if 0 < len(polygon) < 3:
        warnings.append("search_polygon has fewer than 3 vertices")

    for i, p in enumerate(_list(plan, "swarm_manifest")):
        where = f"swarm_manifest[{i}]"
        drone_id = _num(p, "drone_id", where)
        if not (0 <= drone_id <= 255 and drone_id == int(drone_id)):
            raise PlanError(f"{where}: drone_id must be an integer 0-255")
        ip = p.get("ip")
        try:
            addr = ipaddress.IPv4Address(ip)
        except (ipaddress.AddressValueError, ValueError, TypeError):
            raise PlanError(f"{where}: 'ip' must be a dotted IPv4 address, got {ip!r}") from None
        if not str(addr).startswith("192.168."):
            # Channel B only carries octets 3-4 (ICD Sec4.6 gap note).
            warnings.append(f"{where}: Channel B upload can only carry 192.168.x.x addresses")

    if "arm_authorised" in plan and not isinstance(plan["arm_authorised"], bool):
        raise PlanError("'arm_authorised' must be true/false")
    if "low_battery_threshold" in plan:
        t = _num(plan, "low_battery_threshold", "plan")
        if not 0.0 <= t <= 1.0:
            raise PlanError("'low_battery_threshold' is a fraction 0.0-1.0")
    if "offboard_phase_mask" in plan:
        m = _num(plan, "offboard_phase_mask", "plan")
        if not (0 <= m <= 0xFFFF and m == int(m)):
            raise PlanError("'offboard_phase_mask' must be an integer 0-65535")
    if "skip_phase_mask" in plan:
        m = _num(plan, "skip_phase_mask", "plan")
        if m not in defs.SKIP_PHASE_MASKS:
            allowed = ", ".join(f"{k} ({v})" for k, v in defs.SKIP_PHASE_MASKS.items())
            raise PlanError(f"'skip_phase_mask' must be one of {allowed}")
    return warnings


def parse(text: str) -> tuple[dict, list[str]]:
    try:
        plan = json.loads(text)
    except json.JSONDecodeError as e:
        raise PlanError(f"invalid JSON: {e}") from None
    return plan, validate(plan)


def dumps(plan: dict) -> str:
    return json.dumps(plan, indent=2) + "\n"


def stamp(plan: dict, created: bool) -> dict:
    now = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    out = dict(plan)
    if created or "created_at" not in out:
        out["created_at"] = now
    out["updated_at"] = now
    return out


def to_mission_items(plan: dict) -> list[MissionItem]:
    """Encode `plan` as Channel B MC_MISSION_UPLOAD items, matching
    gcs_mavlink_node.cpp's decode_*_item() functions field for field.

    Fields Channel B cannot carry (arm_authorised, low_battery_threshold,
    offboard_phase_mask) are silently dropped -- documented MC-side gap.
    skip_phase_mask goes as a trailing MC_MISSION_CONFIG item when nonzero.
    """
    validate(plan)
    items: list[MissionItem] = []
    for wp in plan.get("ingress_waypoints", []):
        # x/y = lat/lon x1e7, z = alt (m), param1 = speed (decoded, not stored MC-side).
        items.append(
            MissionItem(
                command=defs.CMD_INGRESS_WAYPOINT,
                frame=defs.MAV_FRAME_GLOBAL_INT,
                param1=float(wp.get("speed_mps", 0.0)),
                x=round(wp["lat"] * 1e7),
                y=round(wp["lon"] * 1e7),
                z=float(wp["alt_m"]),
            )
        )
    for fo in plan.get("formation_offsets", []):
        # param1 = drone_id, x/y = N/E in mm, z = D in m (native float).
        items.append(
            MissionItem(
                command=defs.CMD_FORMATION_OFFSET,
                frame=defs.MAV_FRAME_LOCAL_NED,
                param1=float(fo["drone_id"]),
                x=round(fo["north_m"] * 1000.0),
                y=round(fo["east_m"] * 1000.0),
                z=float(fo["down_m"]),
            )
        )
    polygon = plan.get("search_polygon", [])
    for i, v in enumerate(polygon):
        items.append(
            MissionItem(
                command=defs.CMD_SEARCH_AREA_VERTEX,
                frame=defs.MAV_FRAME_GLOBAL_INT,
                param1=float(i),
                param2=float(len(polygon)),
                x=round(v["lat"] * 1e7),
                y=round(v["lon"] * 1e7),
            )
        )
    for p in plan.get("swarm_manifest", []):
        octets = str(ipaddress.IPv4Address(p["ip"])).split(".")
        # x = drone_id, y = octet 3, z = octet 4 (MC rebuilds 192.168.<y>.<z>).
        items.append(
            MissionItem(
                command=defs.CMD_SWARM_PEER_ENTRY,
                frame=defs.MAV_FRAME_MISSION,
                x=int(p["drone_id"]),
                y=int(octets[2]),
                z=float(octets[3]),
            )
        )
    if plan.get("skip_phase_mask", 0):
        items.append(
            MissionItem(
                command=defs.CMD_MISSION_CONFIG,
                frame=defs.MAV_FRAME_MISSION,
                param1=float(plan["skip_phase_mask"]),
            )
        )
    return items


def example_plan(name: str = "bench_test") -> dict:
    """A small, valid plan with placeholder coordinates -- a starting point
    for the editor, not a flyable mission. Replace lat/lon with the test
    site's own before activating it on a real drone."""
    return {
        "name": name,
        "ingress_waypoints": [
            {"lat": 1.3521, "lon": 103.8198, "alt_m": 30.0, "speed_mps": 5.0},
            {"lat": 1.3525, "lon": 103.8203, "alt_m": 30.0, "speed_mps": 5.0},
        ],
        "formation_offsets": [{"drone_id": 1, "north_m": 0.0, "east_m": 0.0, "down_m": 0.0}],
        "search_polygon": [
            {"lat": 1.3530, "lon": 103.8200},
            {"lat": 1.3530, "lon": 103.8210},
            {"lat": 1.3522, "lon": 103.8210},
            {"lat": 1.3522, "lon": 103.8200},
        ],
        "swarm_manifest": [{"drone_id": 1, "ip": "192.168.100.11"}],
        "arm_authorised": False,
        "low_battery_threshold": 0.2,
        "offboard_phase_mask": 0,
        "skip_phase_mask": 0,
    }
