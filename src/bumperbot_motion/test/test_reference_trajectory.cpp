#include <cmath>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "bumperbot_motion/reference_trajectory.hpp"
#include "tf2/utils.h"

TEST(ReferenceTrajectory, interpolates_straight_path)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  for (int i = 0; i <= 10; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0.1 * i;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  geometry_msgs::msg::PoseStamped robot;
  robot.pose.position.x = 0.2;
  bumperbot_motion::ReferenceTrajectory generator;
  const auto reference = generator.generate(path, robot, 10, 0.05, 0.15);
  ASSERT_EQ(reference.size(), 11U);
  EXPECT_NEAR(reference.front().pose.pose.position.x, 0.2, 1e-9);
  EXPECT_NEAR(reference[1].pose.pose.position.x, 0.2075, 1e-9);
  EXPECT_DOUBLE_EQ(reference.front().linear_velocity, 0.15);
  EXPECT_NEAR(reference.front().angular_velocity, 0.0, 1e-9);
}

TEST(ReferenceTrajectory, starts_at_projection_on_nearest_segment)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  for (int i = 0; i <= 2; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = static_cast<double>(i);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  geometry_msgs::msg::PoseStamped robot;
  robot.pose.position.x = 0.75;
  robot.pose.position.y = 0.20;

  bumperbot_motion::ReferenceTrajectory generator;
  const auto reference = generator.generate(path, robot, 10, 0.05, 0.15);

  ASSERT_EQ(reference.size(), 11U);
  EXPECT_NEAR(reference.front().pose.pose.position.x, 0.75, 1e-9);
  EXPECT_NEAR(reference.front().pose.pose.position.y, 0.0, 1e-9);
  EXPECT_GT(reference[1].pose.pose.position.x, reference.front().pose.pose.position.x);
}

TEST(ReferenceTrajectory, handles_single_pose_path)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 1.0;
  pose.pose.position.y = -0.5;
  pose.pose.orientation.w = 1.0;
  path.poses.push_back(pose);

  bumperbot_motion::ReferenceTrajectory generator;
  const auto reference = generator.generate(
    path, geometry_msgs::msg::PoseStamped{}, 4, 0.05, 0.15);

  ASSERT_EQ(reference.size(), 5U);
  for (const auto & point : reference) {
    EXPECT_DOUBLE_EQ(point.pose.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(point.pose.pose.position.y, -0.5);
    EXPECT_DOUBLE_EQ(point.linear_velocity, 0.0);
    EXPECT_DOUBLE_EQ(point.angular_velocity, 0.0);
  }
}

TEST(ReferenceTrajectory, handles_duplicate_consecutive_poses)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  for (const double x : {0.0, 0.5, 0.5, 1.0}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  geometry_msgs::msg::PoseStamped robot;
  robot.pose.position.x = 0.5;

  bumperbot_motion::ReferenceTrajectory generator;
  const auto reference = generator.generate(path, robot, 10, 0.05, 0.15);

  ASSERT_EQ(reference.size(), 11U);
  for (const auto & point : reference) {
    EXPECT_TRUE(std::isfinite(point.pose.pose.position.x));
    EXPECT_TRUE(std::isfinite(point.pose.pose.position.y));
  }
}

TEST(ReferenceTrajectory, handles_robot_before_middle_and_after_path)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  for (int i = 0; i <= 2; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = static_cast<double>(i);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  bumperbot_motion::ReferenceTrajectory generator;
  geometry_msgs::msg::PoseStamped robot;

  robot.pose.position.x = -1.0;
  const auto before = generator.generate(path, robot, 2, 0.05, 0.15);
  robot.pose.position.x = 0.75;
  const auto middle = generator.generate(path, robot, 2, 0.05, 0.15);
  robot.pose.position.x = 3.0;
  const auto after = generator.generate(path, robot, 2, 0.05, 0.15);

  ASSERT_EQ(before.size(), 3U);
  ASSERT_EQ(middle.size(), 3U);
  ASSERT_EQ(after.size(), 3U);
  EXPECT_NEAR(before.front().pose.pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(middle.front().pose.pose.position.x, 0.75, 1e-9);
  EXPECT_NEAR(after.front().pose.pose.position.x, 2.0, 1e-9);
  EXPECT_DOUBLE_EQ(after.front().linear_velocity, 0.0);
}

TEST(ReferenceTrajectory, interpolates_yaw_across_pi_boundary)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  double x = 0.0;
  double y = 0.0;
  for (int i = 0; i < 3; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    const double yaw = i == 0 ? 179.0 * M_PI / 180.0 : -179.0 * M_PI / 180.0;
    pose.pose.orientation.z = std::sin(0.5 * yaw);
    pose.pose.orientation.w = std::cos(0.5 * yaw);
    path.poses.push_back(pose);
    const double segment_yaw = i == 0 ? 179.0 * M_PI / 180.0 : -179.0 * M_PI / 180.0;
    x += std::cos(segment_yaw);
    y += std::sin(segment_yaw);
  }
  geometry_msgs::msg::PoseStamped robot;
  robot.pose = path.poses.front().pose;

  bumperbot_motion::ReferenceTrajectory generator;
  const auto reference = generator.generate(path, robot, 2, 0.05, 10.0);

  ASSERT_EQ(reference.size(), 3U);
  EXPECT_NEAR(std::abs(tf2::getYaw(reference[1].pose.pose.orientation)), M_PI, 1e-6);
}

TEST(ReferenceTrajectory, creates_finite_reference_for_curve)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  for (int i = 0; i <= 10; ++i) {
    const double angle = 0.5 * M_PI * static_cast<double>(i) / 10.0;
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = std::sin(angle);
    pose.pose.position.y = 1.0 - std::cos(angle);
    pose.pose.orientation.z = std::sin(0.5 * angle);
    pose.pose.orientation.w = std::cos(0.5 * angle);
    path.poses.push_back(pose);
  }
  geometry_msgs::msg::PoseStamped robot;
  robot.pose = path.poses.front().pose;

  bumperbot_motion::ReferenceTrajectory generator;
  const auto reference = generator.generate(path, robot, 10, 0.05, 0.15);

  ASSERT_EQ(reference.size(), 11U);
  for (const auto & point : reference) {
    EXPECT_TRUE(std::isfinite(point.angular_velocity));
    EXPECT_GE(point.angular_velocity, 0.0);
  }
}

TEST(ReferenceTrajectory, self_intersection_selects_goal_side_branch)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  for (const auto & xy : std::vector<std::pair<double, double>>{
    {-1.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}, {0.0, 0.0}, {0.0, 1.0}})
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = xy.first;
    pose.pose.position.y = xy.second;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  geometry_msgs::msg::PoseStamped robot;
  robot.pose.position.x = 0.0;
  robot.pose.position.y = 0.0;

  bumperbot_motion::ReferenceTrajectory generator;
  const auto reference = generator.generate(path, robot, 2, 0.1, 0.5);

  ASSERT_EQ(reference.size(), 3U);
  EXPECT_NEAR(reference.front().pose.pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(reference.front().pose.pose.position.y, 0.0, 1e-9);
  EXPECT_NEAR(reference[1].pose.pose.position.x, 0.0, 1e-9);
  EXPECT_GT(reference[1].pose.pose.position.y, 0.0);
}

TEST(ReferenceTrajectory, progress_window_prevents_large_backward_jump)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  for (int i = 0; i <= 10; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = static_cast<double>(i);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  bumperbot_motion::ReferenceTrajectory generator;
  generator.setPlan(path);
  geometry_msgs::msg::PoseStamped robot;
  robot.pose.position.x = 8.2;
  const auto forward = generator.generate(path, robot, 2, 0.05, 0.15);
  robot.pose.position.x = 0.1;
  const auto localization_jump = generator.generate(path, robot, 2, 0.05, 0.15);

  ASSERT_FALSE(forward.empty());
  ASSERT_FALSE(localization_jump.empty());
  EXPECT_GT(forward.front().arc_length, 8.0);
  EXPECT_GE(localization_jump.front().arc_length, 5.0);
}

TEST(ReferenceTrajectory, speed_profile_respects_acceleration_and_brakes_near_goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  for (int i = 0; i <= 100; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0.01 * static_cast<double>(i);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  bumperbot_motion::ReferenceTrajectory generator;
  geometry_msgs::msg::PoseStamped robot;
  robot.pose.position.x = 0.0;
  const auto accelerating = generator.generate(
    path, robot, 5, 0.05, 0.15, 0.0, 0.5, 0.20, 0.8);

  ASSERT_EQ(accelerating.size(), 6U);
  EXPECT_NEAR(accelerating.front().linear_velocity, 0.025, 1e-9);
  for (std::size_t k = 1U; k < accelerating.size(); ++k) {
    EXPECT_LE(
      std::abs(accelerating[k].linear_velocity - accelerating[k - 1U].linear_velocity),
      0.025 + 1e-9);
  }

  generator.reset();
  robot.pose.position.x = 0.99;
  const auto braking = generator.generate(
    path, robot, 5, 0.05, 0.15, 0.15, 0.5, 0.20, 0.8);
  ASSERT_FALSE(braking.empty());
  EXPECT_LT(braking.front().linear_velocity, 0.15);
}

TEST(ReferenceTrajectory, curvature_profile_limits_angular_velocity)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  for (int i = 0; i <= 20; ++i) {
    const double angle = M_PI * static_cast<double>(i) / 20.0;
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0.10 * std::sin(angle);
    pose.pose.position.y = 0.10 * (1.0 - std::cos(angle));
    pose.pose.orientation.z = std::sin(0.5 * angle);
    pose.pose.orientation.w = std::cos(0.5 * angle);
    path.poses.push_back(pose);
  }
  geometry_msgs::msg::PoseStamped robot;
  robot.pose = path.poses.front().pose;
  bumperbot_motion::ReferenceTrajectory generator;
  const auto reference = generator.generate(
    path, robot, 10, 0.05, 0.15, 0.15, 0.5, 0.15, 0.20);

  ASSERT_EQ(reference.size(), 11U);
  bool speed_was_reduced = false;
  for (const auto & point : reference) {
    speed_was_reduced = speed_was_reduced || point.linear_velocity < 0.15 - 1e-6;
    EXPECT_LE(std::abs(point.angular_velocity), 0.20 + 1e-6);
  }
  EXPECT_TRUE(speed_was_reduced);
}

TEST(ReferenceTrajectory, dynamic_linear_speed_limit_caps_entire_profile)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  for (int i = 0; i <= 100; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0.01 * static_cast<double>(i);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  geometry_msgs::msg::PoseStamped robot;
  robot.pose.orientation.w = 1.0;
  bumperbot_motion::ReferenceTrajectory generator;
  const auto reference = generator.generate(
    path, robot, 10, 0.05, 0.15, 0.05, 0.5, 0.05, 0.8);

  ASSERT_EQ(reference.size(), 11U);
  for (const auto & point : reference) {
    EXPECT_LE(point.linear_velocity, 0.05 + 1e-9);
  }
  EXPECT_NEAR(reference.front().linear_velocity, 0.05, 1e-9);
}

TEST(ReferenceTrajectory, endpoint_orientation_stays_aligned_with_path)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  for (int i = 0; i < 2; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = static_cast<double>(i);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  // Goal yêu cầu quay 90 độ nhưng reference tracking vẫn phải theo trục x.
  path.poses.back().pose.orientation.z = std::sin(0.25 * M_PI);
  path.poses.back().pose.orientation.w = std::cos(0.25 * M_PI);
  geometry_msgs::msg::PoseStamped robot;
  robot.pose.position.x = 1.0;

  bumperbot_motion::ReferenceTrajectory generator;
  const auto reference = generator.generate(path, robot, 2, 0.05, 0.15);

  ASSERT_EQ(reference.size(), 3U);
  for (const auto & point : reference) {
    EXPECT_NEAR(tf2::getYaw(point.pose.pose.orientation), 0.0, 1e-9);
  }
}
