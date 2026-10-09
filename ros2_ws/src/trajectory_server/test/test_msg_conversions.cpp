#include <stdexcept>

#include <gtest/gtest.h>

#include "trajectory_server/msg_conversions.hpp"

using trajectory_server::WaypointTrajectory;

namespace
{

// Moving knots with accelerations, and a turn that runs past the last knot.
WaypointTrajectory example_trajectory()
{
  std::vector<WaypointTrajectory::Knot> knots(3);
  knots[0].position = {1, 2, 2.5};
  knots[1].position = {4, 2, 2.5};
  knots[1].velocity = {0.4, 0.3, 0};
  knots[1].acceleration = {0.1, -0.2, 0.05};
  knots[2].position = {4, 6, 3};
  return WaypointTrajectory(knots, {4.0, 5.5}, 0.3, {{0.5, 2.0, 1.2}, {8.0, 3.0, -2.0}});
}

}  // namespace

TEST(MsgConversions, RoundTripIsExact)
{
  const WaypointTrajectory original = example_trajectory();
  const auto restored = trajectory_server::from_msg(trajectory_server::to_msg(original));

  ASSERT_EQ(restored->knots().size(), original.knots().size());
  EXPECT_EQ(restored->duration(), original.duration());
  for (double t = 0.0; t <= original.duration(); t += 0.05) {
    const auto a = original.sample(t);
    const auto b = restored->sample(t);
    EXPECT_EQ(a.position, b.position) << "t = " << t;
    EXPECT_EQ(a.velocity, b.velocity) << "t = " << t;
    EXPECT_EQ(a.acceleration, b.acceleration) << "t = " << t;
    EXPECT_EQ(a.yaw, b.yaw) << "t = " << t;
    EXPECT_EQ(a.yaw_rate, b.yaw_rate) << "t = " << t;
  }
}

TEST(MsgConversions, RejectsInvalidMessages)
{
  auto msg = trajectory_server::to_msg(example_trajectory());
  msg.durations.pop_back();
  EXPECT_THROW(trajectory_server::from_msg(msg), std::invalid_argument);

  msg = trajectory_server::to_msg(example_trajectory());
  msg.knots.back().velocity.x = 1.0;  // must end at rest
  EXPECT_THROW(trajectory_server::from_msg(msg), std::invalid_argument);

  EXPECT_THROW(
    trajectory_server::from_msg(drone_interfaces::msg::WaypointTrajectory{}),
    std::invalid_argument);
}
