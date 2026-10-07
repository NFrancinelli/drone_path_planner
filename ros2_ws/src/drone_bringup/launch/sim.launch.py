"""PX4 SITL + headless Gazebo Harmonic + XRCE-DDS agent + state bridge + RViz + trajectory server.

    ros2 launch drone_bringup sim.launch.py               # headless Gazebo, RViz
    ros2 launch drone_bringup sim.launch.py gui:=true     # also open the (heavy) Gazebo GUI
    ros2 launch drone_bringup sim.launch.py rviz:=false   # nothing on screen (CI, benchmarks)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PX4_DIR = os.environ.get("PX4_DIR", "/opt/PX4-Autopilot")

# PX4 airframe ids for the Gazebo models we use.
AIRFRAMES = {"gz_x500": "4001", "gz_x500_depth": "4002"}


def generate_launch_description():
    model = LaunchConfiguration("model")
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    mission = LaunchConfiguration("mission")

    agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        name="xrce_agent",
        output="log",
    )

    # PX4 starts the Gazebo server itself, and the GUI too unless HEADLESS is set.
    px4 = ExecuteProcess(
        cmd=[os.path.join(PX4_DIR, "build/px4_sitl_default/bin/px4"), "-d"],
        cwd=PX4_DIR,
        additional_env={
            "PX4_SIM_MODEL": model,
            "PX4_SYS_AUTOSTART": PythonExpression([str(AIRFRAMES), ".get('", model, "', '4001')"]),
            "PX4_GZ_WORLD": world,
            "HEADLESS": PythonExpression(["'' if '", gui, "'.lower() == 'true' else '1'"]),
        },
        name="px4",
        output="screen",
    )

    # No ground station in this setup, so PX4 must not require (or failsafe on) a GCS link.
    # This can't go through PX4_PARAM_* env vars: those run before the airframe script, and
    # setting a param to its firmware default is a no-op, so the airframe's
    # `param set-default NAV_DLL_ACT 2` would win. Set it after the airframe script instead;
    # rcS starts the DDS client after the airframe, so a running client is the signal.
    px4_params = ExecuteProcess(
        cmd=["bash", "-c",
             "for i in $(seq 60); do "
             "./bin/px4-uxrce_dds_client status >/dev/null 2>&1 "
             "&& ./bin/px4-param set NAV_DLL_ACT 0 && exit 0; "
             "sleep 1; done; echo 'failed to set PX4 params' >&2; exit 1"],
        cwd=os.path.join(PX4_DIR, "build/px4_sitl_default"),
        name="px4_params",
        output="screen",
    )

    state_bridge = Node(
        package="px4_state_bridge",
        executable="px4_state_bridge_node",
        name="px4_state_bridge",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("drone_bringup"), "rviz", "sim.rviz"])],
        output="log",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    trajectory_server = Node(
        package="trajectory_server",
        executable="trajectory_server_node",
        name="trajectory_server",
        output="screen",
        condition=IfCondition(mission),
    )

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="gz_x500", description=f"one of {list(AIRFRAMES)}"),
        DeclareLaunchArgument("world", default_value="default", description="Gazebo world in PX4's worlds dir"),
        DeclareLaunchArgument("gui", default_value="false", description="open the Gazebo GUI"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("mission", default_value="true", description="run the figure-eight mission"),
        agent,
        px4,
        px4_params,
        state_bridge,
        rviz,
        # Give PX4 a head start; the node waits for PX4 anyway.
        TimerAction(period=5.0, actions=[trajectory_server]),
    ])
