# Smooth trajectory through knots: segment k is the quintic Hermite interpolant between
# knots k and k + 1. Mirrors trajectory_server::WaypointTrajectory, which rebuilds it exactly.
# header.frame_id: world frame (ENU). header.stamp: earliest time to start flying it.
std_msgs/Header header
TrajectoryKnot[] knots   # first and last at rest
float64[] durations      # [s] durations[k]: time from knot k to knot k + 1
float64 initial_yaw      # [rad] ENU heading at the start
YawTurn[] turns          # sorted, not overlapping
