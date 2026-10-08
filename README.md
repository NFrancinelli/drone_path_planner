# 3D Planner for an Autonomous Drone

A from-scratch 3D path planner that lets a PX4 drone fly to a goal through a space it has
never seen: A* on a live OctoMap, a smoothed trajectory, and replanning as the map grows.
Simulation, flight control and mapping are off-the-shelf (Gazebo Harmonic, PX4 SITL, OctoMap).

Project page: https://nfrancinelli.github.io/projects/3d-planner/

## Status

- [x] Milestone 1 – infrastructure: Docker stack, PX4 offboard control, trajectory server flying a figure-eight
- [x] Milestone 2 – depth camera → OctoMap
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
ros2 launch drone_bringup sim.launch.py localization:=gps   # PX4's GPS/compass estimate
```

Gazebo runs headless by default (sensors render offscreen on the GPU); RViz shows the drone,
the commanded trajectory (orange), the flown trail (green) and the OctoMap built from the
depth camera. The default world is `boxes`; pick another with `world:=<name>`.

Run `docker/run.sh` again from another terminal to attach a second shell to the same container.

## Localization

By default (`localization:=mocap`) the drone is localized by a **simulated motion-capture
system**, as in a lab flight arena: Gazebo's true pose is fed to PX4 as external vision
(GPS, compass and barometer are not fused), and the same pose drives the map's TF. This
isolates the planner, which is what the project evaluates, from state-estimation error.
Flying the planner on onboard estimation (e.g. visual-inertial SLAM) is a natural next step.

`localization:=gps` keeps PX4's stock outdoor estimate for comparison. Measured against
ground truth during the figure-eight, its heading error grows with turn rate (p95 3° below
0.2 rad/s, 17° above 0.6 rad/s), which smears obstacles by up to ~2 m at 8 m range:

| figure-eight, `boxes` world | mocap | gps |
|---|---|---|
| map voxels within 0.3 m of a real obstacle | 100 % | ~73 % |

## Layout

| Path | What |
|---|---|
| `docker/` | Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + PX4 v1.17.0 + XRCE-DDS agent |
| `ros2_ws/src/trajectory_server` | Streams setpoints to PX4 in offboard mode; ROS-free trajectory core with unit tests |
| `ros2_ws/src/px4_state_bridge` | PX4 odometry → `/odom`, TF, flown trail, drone model for RViz; `mocap_bridge` feeds ground truth to PX4 as external vision |
| `ros2_ws/src/drone_sim` | Gazebo models and worlds: `x500_depth_lite` (x500 + one 320×240 @ 10 Hz depth camera, pitched up 12°), `boxes` test world |
| `ros2_ws/src/drone_bringup` | Launch files, RViz config |

## Conventions

Everything in ROS uses ENU / FLU. Conversions to and from PX4's NED / FRD live in exactly one
place, `trajectory_server/frames.hpp`, and are unit-tested.

Time: px4_msgs timestamps are in the XRCE agent's system clock, while ROS runs on Gazebo's sim
time. `px4_state_bridge/px4_time.hpp` converts between them exactly through PX4's timesync
offset (in lockstep SITL, PX4's clock *is* sim time). Messages sent to PX4 by the trajectory
server are stamped with system time.

## Known limitations

- Localization is simulated motion capture by default (see above); robustness to onboard
  state-estimation error is not evaluated yet.
- Single forward-facing depth camera: test worlds contain no obstacles directly above or
  below the flight path.
