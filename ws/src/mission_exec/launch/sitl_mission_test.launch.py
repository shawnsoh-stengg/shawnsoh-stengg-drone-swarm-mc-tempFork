"""
sitl_mission_test.launch.py -- TEST-002 SITL harness (TP-MC-001 §6, TP-ST).

The `ros2 launch mission_exec sitl_mission_test.launch.py` entry point
CLAUDE.md/docs/GETTING-STARTED.md/docs/TESTING.md already document. Brings
up `num_drones` (1-3) full single-drone MC stacks against PX4 SITL, each
isolated on its own ROS_DOMAIN_ID (see sitl_common.py's own docstring for
why), for manual/interactive use -- e.g. watch it with
`ROS_DOMAIN_ID=20 ros2 topic echo /mc/mission_phase` in another terminal
for drone 1. For automated scenarios, see
`test/test_sitl_mission_scenarios.py` instead, which reuses this same
module's `build_drone_nodes()` rather than this file.

Requires a built PX4-Autopilot checkout: set PX4_DIR (or pass
`px4_dir:=<path>`) to a checkout already built with `make px4_sitl_default`
(TP-MC-001 §3.2). Real Gazebo rendering is available via `use_gazebo:=true`
for a developer who wants a visual run; the default (`sihsim`) needs no
GPU/display.
"""
import os
import sys
import tempfile

import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sitl_common  # noqa: E402


def _launch_setup(context, *args, **kwargs):
    px4_dir = LaunchConfiguration('px4_dir').perform(context)
    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    use_gazebo = LaunchConfiguration('use_gazebo').perform(context).lower() == 'true'

    sitl_common.require_valid_px4_dir(px4_dir)
    scratch_dir = tempfile.mkdtemp(prefix='sitl_mission_test_')

    actions = []
    for index in range(num_drones):
        actions.extend(
            sitl_common.build_drone_nodes(index, px4_dir, scratch_dir, use_gazebo=use_gazebo))
    return actions


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'px4_dir', default_value=os.environ.get('PX4_DIR', ''),
            description='Path to a built PX4-Autopilot checkout (make px4_sitl_default).'),
        DeclareLaunchArgument(
            'num_drones', default_value='1',
            description='Number of SITL drone instances to bring up (1-3, TP-MC-001 §3.2).'),
        DeclareLaunchArgument(
            'use_gazebo', default_value='false',
            description='true for real Gazebo rendering; false (default) uses PX4 SIH.'),
        OpaqueFunction(function=_launch_setup),
    ])
