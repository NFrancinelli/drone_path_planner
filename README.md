# 3D Planner for an Autonomous Drone

A from-scratch 3D path planner that lets a PX4 drone fly to a goal through a space it has
never seen: A* on a live OctoMap, a smoothed trajectory, and replanning as the map grows.
Simulation, flight control and mapping are off-the-shelf (Gazebo Harmonic, PX4 SITL, OctoMap).

Project page: https://nfrancinelli.github.io/projects/3d-planner/

## Status

- [x] Milestone 1 – infrastructure: Docker stack, PX4 offboard control, trajectory server flying a figure-eight
- [ ] Milestone 2 – depth camera → OctoMap
- [ ] Milestone 3 – A* with unknown-space cost, smoothing, collision check
- [ ] Milestone 4 – replanning in the factory world
- [ ] Milestone 5 – unknown-cost comparison + demo video

## Quick start

Requires Docker (and the NVIDIA container toolkit for GPU rendering).

```bash
docker/build.sh                       # once; builds PX4 + px4_msgs, takes a while
docker/run.sh                         # shell inside the container, repo's ros2_ws mounted at /ws
colcon build --symlink-install && source install/setup.bash
ros2 launch drone_bringup sim.launch.py               # headless Gazebo + RViz (default)
ros2 launch drone_bringup sim.launch.py gui:=true     # also open the Gazebo GUI (heavy)
ros2 launch drone_bringup sim.launch.py rviz:=false   # nothing on screen
```

Gazebo runs headless by default; RViz shows the drone, the commanded trajectory (orange)
and the flown trail (green).

Run `docker/run.sh` again from another terminal to attach a second shell to the same container.

## Layout

| Path | What |
|---|---|
| `docker/` | Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + PX4 v1.17.0 + XRCE-DDS agent |
| `ros2_ws/src/trajectory_server` | Streams setpoints to PX4 in offboard mode; ROS-free trajectory core with unit tests |
| `ros2_ws/src/px4_state_bridge` | PX4 odometry → `/odom`, TF `odom → base_link`, flown trail, drone model for RViz |
| `ros2_ws/src/drone_bringup` | Launch files, RViz config |

## Conventions

Everything in ROS uses ENU / FLU. Conversions to and from PX4's NED / FRD live in exactly one
place, `trajectory_server/frames.hpp`, and are unit-tested.

## Known limitations

- Single forward-facing depth camera: test worlds contain no obstacles directly above or
  below the flight path.
