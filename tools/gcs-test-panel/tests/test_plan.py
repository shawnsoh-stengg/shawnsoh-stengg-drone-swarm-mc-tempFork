import pytest

from gcs_test_panel.core import defs, plan


def test_example_plan_is_valid():
    assert plan.validate(plan.example_plan()) == []


def test_name_required():
    with pytest.raises(plan.PlanError):
        plan.validate({"ingress_waypoints": []})


def test_bad_field_type_rejected():
    p = plan.example_plan()
    p["ingress_waypoints"][0]["lat"] = "1.3"
    with pytest.raises(plan.PlanError, match="lat"):
        plan.validate(p)


def test_null_island_warns():
    p = plan.example_plan()
    p["ingress_waypoints"][0].update(lat=0, lon=0)
    assert any("Null Island" in w for w in plan.validate(p))


def test_items_match_gcs_mavlink_node_decoding():
    p = plan.example_plan()
    items = plan.to_mission_items(p)
    wp = items[0]
    assert wp.command == defs.CMD_INGRESS_WAYPOINT
    assert (wp.x, wp.y, wp.z, wp.param1) == (13521000, 1038198000, 30.0, 5.0)

    fo = items[2]
    assert fo.command == defs.CMD_FORMATION_OFFSET and fo.param1 == 1

    p["formation_offsets"][0].update(north_m=1.25, east_m=-2.5, down_m=-0.5)
    fo = plan.to_mission_items(p)[2]
    assert (fo.x, fo.y, fo.z) == (1250, -2500, -0.5)  # N/E in mm, D native metres

    verts = [i for i in items if i.command == defs.CMD_SEARCH_AREA_VERTEX]
    assert [(v.param1, v.param2) for v in verts] == [(0, 4), (1, 4), (2, 4), (3, 4)]

    peer = items[-1]
    assert peer.command == defs.CMD_SWARM_PEER_ENTRY
    assert (peer.x, peer.y, peer.z) == (1, 100, 11.0)  # 192.168.<y>.<z>


def test_skip_phase_mask_only_accepts_flyable_stage_sets():
    p = plan.example_plan()
    for mask in defs.SKIP_PHASE_MASKS:
        p["skip_phase_mask"] = mask
        plan.validate(p)
    # MUSTER alone; solo (refused until search_planner_node exists); a float.
    for bad in (0x10, defs.SKIP_SWARM_PHASES, 240.0):
        p["skip_phase_mask"] = bad
        with pytest.raises(plan.PlanError, match="skip_phase_mask"):
            plan.validate(p)


def test_skip_phase_mask_sent_as_trailing_mission_config_item():
    p = plan.example_plan()
    assert all(i.command != defs.CMD_MISSION_CONFIG for i in plan.to_mission_items(p))

    p["skip_phase_mask"] = defs.SKIP_SWARM_PHASES | defs.SKIP_SEARCH
    cfg = plan.to_mission_items(p)[-1]
    assert cfg.command == defs.CMD_MISSION_CONFIG
    assert cfg.param1 == 0xF0


def test_example_plan_files_are_valid():
    # examples/*.json: one plan per skip_phase_mask stage set (README "Example plans").
    from pathlib import Path

    files = sorted((Path(__file__).parent.parent / "examples").glob("*.json"))
    masks = set()
    for f in files:
        p, warnings = plan.parse(f.read_text())
        assert warnings == [], (f.name, warnings)
        assert p["name"] == f.stem
        masks.add(p.get("skip_phase_mask", 0))
    assert masks == set(defs.SKIP_PHASE_MASKS)
