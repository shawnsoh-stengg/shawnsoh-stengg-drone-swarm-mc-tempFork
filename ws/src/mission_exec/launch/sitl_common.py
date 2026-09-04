"""
Shared node-generation logic for TEST-002's SITL harness.

Used by both `sitl_mission_test.launch.py` (the standalone, manually
`ros2 launch`-able entry point CLAUDE.md/docs/GETTING-STARTED.md already
document) and `test/test_sitl_mission_scenarios.py` (the automated pytest
suite's `generate_test_description()`), so the two never drift out of sync
on what one drone's MC stack actually looks like under SITL.

Simulator backend: PX4's built-in SIH (simulation-in-hardware) dynamics
model, not Gazebo, for every *automated* scenario -- SIH needs no
rendering/GPU and is deterministic, which is what makes a CI job
tractable. `use_gazebo=True` is wired through for a developer who wants a
visual/physics-realistic manual run of the standalone launch file, but the
automated suite never exercises that path (see PROJECT-PLAN-5-MONTH.md's
TEST-002 entry for the full reasoning).

Multi-drone isolation: this system's real deployment isolates drones by
running each on its own physical host (separate DDS domain by virtue of
being a separate machine, cross-host traffic bridged only via
zenoh-bridge-dds for specific /mc/swarm/* topics -- CLAUDE.md's own
architecture notes). A single `ros2 launch` invocation can't spread nodes
across hosts, so each drone instance here instead gets its own
`ROS_DOMAIN_ID` -- the closest same-host equivalent of "separate DDS
domain per drone" production actually relies on. Only instance 0 is
exercised by the automated pytest suite (real multi-drone swarm
coordination is moot while swarm_coord/search_planner_node are stubs, see
mock_swarm_coord_node.py) -- instances 1+ are wired for TP-MC-001 §3.2's
addressing scheme but not runtime-verified here.
"""
import os

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

# TP-MC-001 §3.2: instance N's GCS-facing address is 127.0.0.1(11+N) --
# QGroundControl/a real GCS would use this to tell drones apart. The
# internal fcc_bridge<->PX4 SITL link (this harness's own concern) instead
# uses one port-offset scheme per instance on plain 127.0.0.1, the same
# mechanism PX4's own Tools/simulation/sitl_multiple_run.sh uses -- binding
# multiple SITL instances to *distinct addresses* on the same port would
# still collide at the OS socket layer for any listener on INADDR_ANY.
GCS_FACING_ADDRESS_BASE_OCTET = 11
FCC_PORT_BASE = 14540
GCS_LOOPBACK_PORT_BASE = 14560
ROS_DOMAIN_ID_BASE = 20
PORT_STRIDE = 10


def px4_binary_path(px4_dir):
    return os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')


def px4_rcs_dir(px4_dir):
    return os.path.join(px4_dir, 'build', 'px4_sitl_default', 'etc')


def require_valid_px4_dir(px4_dir):
    """Raise a clear, actionable error if px4_dir isn't a built PX4-Autopilot checkout."""
    binary = px4_binary_path(px4_dir)
    if not os.path.isfile(binary):
        raise RuntimeError(
            'sitl_mission_test.launch.py: PX4_DIR={!r} has no built SITL binary at {} -- '
            'set the PX4_DIR environment variable (or pass px4_dir:=<path>) to a '
            'PX4-Autopilot checkout already built with "make px4_sitl_default" '
            '(TP-MC-001 §3.2).'.format(px4_dir, binary)
        )


def build_drone_nodes(
    index, px4_dir, scratch_dir, use_gazebo=False, mission_upload_path=None,
    extra_mission_manager_params=None,
):
    """
    Return the list of launch Actions for one drone's full SITL MC stack.

    `index` is 0-based (drone_id = index + 1). Every process in the
    returned list shares one ROS_DOMAIN_ID, isolating this drone's ROS
    graph from any other instance's in the same `ros2 launch` invocation
    (see this module's own docstring for why).

    `extra_mission_manager_params`, if given, is layered on top of
    `mission_manager_node`'s normal params -- used ONLY by the automated
    pytest suite to route around a real, separate bug this harness
    surfaced (not a legitimate flight parameter, see
    test_sitl_mission_scenarios.py's own comment on it and
    PROJECT-PLAN-5-MONTH.md's TEST-002 entry): `action_send_takeoff()`
    sends MAV_CMD_NAV_TAKEOFF's altitude as an absolute AMSL value, but
    `guard_takeoff_to_ingress()` checks it against relative-to-home NED
    altitude -- PX4 SITL's non-zero default home elevation means the
    guard's tolerance is never satisfiable as currently written. Left at
    its real, unmodified default for the standalone manual launch file
    (`sitl_mission_test.launch.py` never passes this) so a developer
    running it by hand rediscovers the real bug rather than having it
    silently papered over.
    """
    drone_id = index + 1
    fcc_port = FCC_PORT_BASE + PORT_STRIDE * index
    gcs_port = GCS_LOOPBACK_PORT_BASE + PORT_STRIDE * index
    ros_domain_id = str(ROS_DOMAIN_ID_BASE + index)
    px4_scratch_dir = os.path.join(scratch_dir, 'px4_instance_{}'.format(index))
    os.makedirs(px4_scratch_dir, exist_ok=True)
    if mission_upload_path is None:
        mission_upload_path = os.path.join(
            scratch_dir, 'mission_upload_drone_{}.yaml'.format(drone_id))

    shared_env = {'ROS_DOMAIN_ID': ros_domain_id}

    px4_env = dict(os.environ)
    px4_env.update(shared_env)
    px4_env['PX4_SIM_MODEL'] = 'gz_x500' if use_gazebo else 'sihsim_quadx'
    px4_env['PX4_SIMULATOR'] = 'gz' if use_gazebo else 'sihsim'
    if not use_gazebo:
        px4_env['HEADLESS'] = '1'
    px4_process = launch.actions.ExecuteProcess(
        cmd=[
            px4_binary_path(px4_dir), '-i', str(index),
            '-d', px4_rcs_dir(px4_dir),
            '-w', px4_scratch_dir,
        ],
        env=px4_env,
        output='screen',
        name='px4_sitl_{}'.format(index),
    )

    fcc_bridge_node = launch_ros.actions.Node(
        package='fcc_bridge',
        executable='fcc_bridge_node',
        additional_env=dict(
            shared_env, SIMULATION_MODE='true', SITL_FCC_PORT=str(fcc_port),
            DRONE_ID=str(drone_id)),
        parameters=[
            os.path.join(get_package_share_directory('fcc_bridge'), 'params', 'sitl.yaml'),
        ],
    )

    gcs_mavlink_node = launch_ros.actions.Node(
        package='gcs_comms',
        executable='gcs_mavlink_node',
        additional_env=dict(
            shared_env, DRONE_ID=str(drone_id), GCS_MAVLINK_LOOPBACK_PORT=str(gcs_port)),
    )

    mission_manager_params = [
        os.path.join(get_package_share_directory('mission_exec'), 'params', 'default.yaml'),
        {'mission_upload_path': mission_upload_path},
    ]
    if extra_mission_manager_params:
        mission_manager_params.append(extra_mission_manager_params)

    mission_manager_node = launch_ros.actions.Node(
        package='mission_exec',
        executable='mission_manager_node',
        additional_env=dict(shared_env),
        parameters=mission_manager_params,
    )

    search_planner_node = launch_ros.actions.Node(
        package='mission_exec',
        executable='search_planner_node',
        additional_env=dict(shared_env),
    )

    mock_swarm_coord_node = launch_ros.actions.Node(
        package='mc_test_utils',
        executable='mock_swarm_coord_node',
        additional_env=dict(shared_env),
        parameters=[{'drone_id': drone_id}],
    )

    # Needed for the LOW_BATTERY failsafe scenario -- health_monitor_node,
    # not mission_manager_node/fcc_bridge_node, is what actually watches
    # VehicleState.battery_pct and publishes the fault (HEALTH-003). Its
    # other liveness watches (swarm_coordinator_node, camera_node, etc.)
    # will simply report those nodes as not-alive in this minimal stack --
    # harmless, nothing here depends on node_alive_mask.
    health_monitor_node = launch_ros.actions.Node(
        package='health_monitor',
        executable='health_monitor_node',
        additional_env=dict(shared_env),
        parameters=[
            os.path.join(get_package_share_directory('health_monitor'), 'params', 'default.yaml'),
        ],
    )

    return [
        px4_process, fcc_bridge_node, gcs_mavlink_node, mission_manager_node,
        search_planner_node, mock_swarm_coord_node, health_monitor_node,
    ]
