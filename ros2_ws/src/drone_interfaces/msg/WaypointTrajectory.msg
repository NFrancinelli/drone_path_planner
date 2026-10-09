# Smooth trajectory through knots: segment k is the quintic Hermite interpolant between
# knots k and k + 1. Mirrors trajectory_server::WaypointTrajectory, which rebuilds it exactly.
# header.frame_id: world frame (ENU). header.stamp: when to start flying it. A trajectory
# whose first knot is moving continues the one being flown, taking over at exactly that time.
std_msgs/Header header
TrajectoryKnot[] knots    # last at rest; first at rest, or moving to continue a trajectory
float64[] durations       # [s] durations[k]: time from knot k to knot k + 1
float64 initial_yaw       # [rad] ENU heading at the start
float64 initial_yaw_rate  # [rad/s] at the start; non-zero needs a first turn at t = 0
YawTurn[] turns           # sorted, not overlapping
