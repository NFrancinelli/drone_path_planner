"""Headless Gazebo + PX4 SITL + depth camera + OctoMap + RViz + trajectory server.

    ros2 launch drone_bringup sim.launch.py               # headless Gazebo, RViz
    ros2 launch drone_bringup sim.launch.py gui:=true     # also open the (heavy) Gazebo GUI
    ros2 launch drone_bringup sim.launch.py rviz:=false   # nothing on screen (CI, benchmarks)

Startup order: Gazebo world -> spawn our drone -> PX4 attaches to it (standalone mode).
Everything on the ROS side runs on simulation time from Gazebo's /clock.

Localization (localization:=...):
  mocap  (default) Gazebo's true pose is fed to PX4 as external vision, like a motion-capture
         system in a lab; GPS, compass and barometer are not fused. Isolates the planner from
         state-estimation error.
  gps    PX4's stock GPS + compass + barometer estimate, as on an outdoor drone. Heading error
         reaches ~17 deg in fast turns, which smears the map.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PX4_DIR = os.environ.get("PX4_DIR", "/opt/PX4-Autopilot")
PX4_BUILD = os.path.join(PX4_DIR, "build/px4_sitl_default")

MODEL = "x500_depth_lite"
MODEL_INSTANCE = MODEL + "_0"
PX4_AIRFRAME = "4001"  # gz_x500: same airframe, our model only adds a camera

# base_link -> camera_link. Must match the camera pose in drone_sim/models/x500_depth_lite.
CAMERA_TF = {"x": "0.12", "y": "0", "z": "0.03", "roll": "0", "pitch": "-0.2094", "yaw": "0"}

# EKF2 setup for localization:=mocap: fuse external vision position + yaw, nothing else.
# These all differ from the firmware defaults, so PX4_PARAM_* env overrides apply (see the
# NAV_DLL_ACT note below for why that matters).
MOCAP_PX4_PARAMS = {
    "EKF2_EV_CTRL": "11",      # horizontal position + vertical position + yaw
    "EKF2_HGT_REF": "3",       # vision
    "EKF2_GPS_CTRL": "0",
    "EKF2_MAG_TYPE": "5",      # none
    "EKF2_BARO_CTRL": "0",
    # Lower bounds on the reported mocap noise. Not too small: with ~1 cm, sub-millimetre
    # height innovations already fail EKF2's pre-arm consistency check, intermittently.
    "EKF2_EVP_NOISE": "0.05",
    "EKF2_EVA_NOISE": "0.05",
}


def generate_launch_description():
    world = LaunchConfiguration("world")
    sim_share = get_package_share_directory("drone_sim")
    sim_time = {"use_sim_time": True}

    gz_env = [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", ":".join([
            os.path.join(sim_share, "models"),
            os.path.join(sim_share, "worlds"),
            os.path.join(PX4_DIR, "Tools/simulation/gz/models"),
        ])),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH",
                               os.path.join(PX4_BUILD, "src/modules/simulation/gz_plugins")),
        SetEnvironmentVariable("GZ_SIM_SERVER_CONFIG_PATH",
                               os.path.join(PX4_DIR, "src/modules/simulation/gz_bridge/server.config")),
    ]

    # Server only; sensors render offscreen (EGL), so no window is needed.
    gz_server = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", "--headless-rendering", "-v", "1",
             PathJoinSubstitution([sim_share, "worlds", [world, ".sdf"]])],
        name="gz_server",
        output="screen",
    )
    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-g"],
        name="gz_gui",
        output="log",
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    model_sdf = os.path.join(sim_share, "models", MODEL, "model.sdf")
    spawn = ExecuteProcess(
        cmd=["bash", "-c",
             "for i in $(seq 60); do gz service -i --service /world/$0/scene/info 2>&1 "
             "| grep -q 'Service providers' && break; sleep 1; done; "
             "gz service -s /world/$0/create --reqtype gz.msgs.EntityFactory "
             "--reptype gz.msgs.Boolean --timeout 5000 "
             "--req \"sdf_filename: '$1', name: '$2', allow_renaming: false\"",
             world, model_sdf, MODEL_INSTANCE],
        name="spawn_drone",
        output="screen",
    )

    agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        name="xrce_agent",
        output="log",
    )

    def make_px4(context):
        env = {
            "PX4_SYS_AUTOSTART": PX4_AIRFRAME,
            "PX4_GZ_STANDALONE": "1",
            "PX4_GZ_WORLD": world.perform(context),
            "PX4_GZ_MODEL_NAME": MODEL_INSTANCE,
        }
        if LaunchConfiguration("localization").perform(context) == "mocap":
            env.update({"PX4_PARAM_" + k: v for k, v in MOCAP_PX4_PARAMS.items()})
        return [ExecuteProcess(
            cmd=[os.path.join(PX4_BUILD, "bin/px4"), "-d"],
            cwd=PX4_DIR,
            additional_env=env,
            name="px4",
            output="screen",
        )]

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
        cwd=PX4_BUILD,
        name="px4_params",
        output="screen",
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            f"/model/{MODEL_INSTANCE}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ],
        remappings=[
            ("/depth_camera/points", "/camera/points"),
            (f"/model/{MODEL_INSTANCE}/odometry", "/ground_truth/odom"),
        ],
        parameters=[sim_time],
        output="log",
    )

    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_tf",
        arguments=[a for k, v in CAMERA_TF.items() for a in (f"--{k}", v)]
        + ["--frame-id", "base_link", "--child-frame-id", "camera_link"],
        parameters=[sim_time],
        output="log",
    )

    state_bridge = Node(
        package="px4_state_bridge",
        executable="px4_state_bridge_node",
        name="px4_state_bridge",
        # In mocap mode the mocap bridge owns odom -> base_link.
        parameters=[sim_time, {"publish_tf": PythonExpression(
            ["'", LaunchConfiguration("localization"), "' != 'mocap'"])}],
        output="screen",
    )

    mocap = Node(
        package="px4_state_bridge",
        executable="mocap_bridge_node",
        name="mocap_bridge",
        remappings=[("mocap/odometry", "/ground_truth/odom")],
        parameters=[sim_time],
        output="log",
        condition=LaunchConfigurationEquals("localization", "mocap"),
    )

    octomap = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        parameters=[sim_time, {
            "frame_id": "odom",
            "base_frame_id": "base_link",
            "resolution": LaunchConfiguration("map_resolution"),
            "sensor_model.max_range": 10.0,
            # Only affects the RViz markers: hides the floor (~90% of the cubes) but keeps it in
            # the octree for the planner. Checked against the voxel extent, so it has to be
            # above the whole floor layer (0.0-0.2 m).
            "occupancy_min_z": 0.25,
            # Free-space markers for debugging. Only read at startup.
            "publish_free_space": LaunchConfiguration("show_free_space"),
        }],
        remappings=[("cloud_in", "/camera/points")],
        output="log",
    )

    planner = Node(
        package="path_planner",
        executable="planner_node",
        name="path_planner",
        parameters=[sim_time],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("drone_bringup"), "rviz", "sim.rviz"])],
        parameters=[sim_time],
        output="log",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    trajectory_server = Node(
        package="trajectory_server",
        executable="trajectory_server_node",
        name="trajectory_server",
        parameters=[sim_time, {"figure_eight.enabled": LaunchConfiguration("figure_eight")}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("mission")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="boxes", description="world in drone_sim/worlds"),
        DeclareLaunchArgument("gui", default_value="false", description="open the Gazebo GUI"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("mission", default_value="true", description="run the trajectory server"),
        DeclareLaunchArgument("figure_eight", default_value="true",
                              description="false: take off, hover and fly planner trajectories instead"),
        DeclareLaunchArgument("map_resolution", default_value="0.2", description="OctoMap voxel size [m]"),
        DeclareLaunchArgument("show_free_space", default_value="false",
                              description="publish OctoMap free voxels on /free_cells_vis_array"),
        DeclareLaunchArgument("localization", default_value="mocap", choices=["mocap", "gps"],
                              description="mocap: ground truth fed to PX4; gps: PX4's own estimate"),
        *gz_env,
        gz_server,
        gz_gui,
        spawn,
        agent,
        gz_bridge,
        camera_tf,
        state_bridge,
        mocap,
        octomap,
        planner,
        rviz,
        # PX4 needs the drone to exist before it can attach to it.
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[
            OpaqueFunction(function=make_px4),
            px4_params,
            # Give PX4 a head start; the node waits for PX4 anyway.
            TimerAction(period=5.0, actions=[trajectory_server]),
        ])),
    ])
