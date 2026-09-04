"""Plain (non-launch_test) unit tests for mavlink_gcs_client's item-building helpers."""
import os

from mc_test_utils import mavlink_gcs_client as gcs_client
from mc_test_utils import mavlink_lite as ml

FIXTURE_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'test_data', 'mission_upload_3drone.yaml',
)


def test_ingress_or_muster_item_scales_lat_lon_to_int32():
    item = gcs_client.ingress_or_muster_item(
        gcs_client.CMD_INGRESS_WAYPOINT, lat_deg=1.5, lon_deg=-2.25, alt_m=30.0)
    assert item['command'] == gcs_client.CMD_INGRESS_WAYPOINT
    assert item['x'] == 15000000
    assert item['y'] == -22500000
    assert item['z'] == 30.0


def test_formation_offset_item_scales_ne_to_millimetres():
    item = gcs_client.formation_offset_item(drone_id=2, n_m=1.234, e_m=-0.5, d_m=-30.0)
    assert item['command'] == gcs_client.CMD_FORMATION_OFFSET
    assert item['param1'] == 2.0
    assert item['x'] == 1234
    assert item['y'] == -500
    assert item['z'] == -30.0


def test_swarm_peer_entry_item_x_y_are_real_ints():
    # Regression: x/y are MISSION_ITEM_INT's actual int32 wire fields --
    # struct.pack('i', 1.0) raises "required argument is not an integer",
    # only caught previously by an actual live SITL run (encode() is never
    # exercised end-to-end by the other unit tests in this file).
    item = gcs_client.swarm_peer_entry_item(drone_id=2, ip_octet3=10, ip_octet4=12)
    assert isinstance(item['x'], int)
    assert isinstance(item['y'], int)


def test_load_mission_items_orders_ingress_then_muster_then_polygon_then_peers():
    items = gcs_client.load_mission_items(FIXTURE_PATH, num_drones=2)

    # 2 (ingress+muster) + 3 (polygon) + 2 (formation offsets) + 2 (swarm peers)
    assert len(items) == 9
    assert items[0]['command'] == gcs_client.CMD_INGRESS_WAYPOINT
    assert items[1]['command'] == gcs_client.CMD_MUSTER_POINT
    assert items[2]['command'] == gcs_client.CMD_SEARCH_AREA_VERTEX
    assert items[2]['param2'] == 3.0  # total_vertices
    assert items[5]['command'] == gcs_client.CMD_FORMATION_OFFSET
    assert items[7]['command'] == gcs_client.CMD_SWARM_PEER_ENTRY


def test_load_mission_items_trims_to_num_drones():
    items_one = gcs_client.load_mission_items(FIXTURE_PATH, num_drones=1)
    items_three = gcs_client.load_mission_items(FIXTURE_PATH, num_drones=3)
    # Each extra drone adds one formation_offset + one swarm_manifest item.
    assert len(items_three) - len(items_one) == 4


def test_load_mission_items_all_encode_as_real_mission_item_int_frames():
    # Regression: every item this module builds must actually survive
    # encode_with_fields()'s struct.pack() -- a wrong Python type (float
    # where MISSION_ITEM_INT's wire format needs an int) raises there, not
    # at item-construction time, so exercise the real encode path here
    # instead of only checking field types/values as the tests above do.
    items = gcs_client.load_mission_items(FIXTURE_PATH, num_drones=3)
    for seq, item in enumerate(items):
        frame = ml.encode_with_fields(
            ml.MSG_MISSION_ITEM_INT, seq, 255, 190, {
                'param1': item.get('param1', 0.0),
                'param2': item.get('param2', 0.0),
                'param3': item.get('param3', 0.0),
                'param4': item.get('param4', 0.0),
                'x': item['x'],
                'y': item['y'],
                'z': item.get('z', 0.0),
                'seq': seq,
                'command': item['command'],
                'target_system': 1,
                'target_component': gcs_client.DEFAULT_TARGET_COMPONENT,
                'frame': 5,
                'current': 0,
                'autocontinue': 1,
                'mission_type': gcs_client.MAV_MISSION_TYPE_MC,
            })
        assert frame[0] == ml.MAVLINK_STX
