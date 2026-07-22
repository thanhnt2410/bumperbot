#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "bumperbot_motion/mpc_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace
{

using LifecycleNode = rclcpp_lifecycle::LifecycleNode;

void initializeRos()
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

LifecycleNode::SharedPtr makeNode(const std::string & suffix)
{
  static int node_id = 0;
  return std::make_shared<LifecycleNode>(
    "test_mpc_controller_" + suffix + "_" + std::to_string(node_id++));
}

std::shared_ptr<tf2_ros::Buffer> makeBuffer(const LifecycleNode::SharedPtr & node)
{
  return std::make_shared<tf2_ros::Buffer>(node->get_clock());
}

nav_msgs::msg::Path straightPath(const LifecycleNode::SharedPtr & node)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  path.header.stamp = node->now();
  for (int i = 0; i <= 200; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = 0.01 * static_cast<double>(i);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

geometry_msgs::msg::PoseStamped robotPose(const LifecycleNode::SharedPtr & node)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom";
  pose.header.stamp = node->now();
  pose.pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::msg::Twist measuredVelocity(double linear_velocity)
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = linear_velocity;
  return velocity;
}

void expectInvalidParameter(
  const std::string & suffix, const std::string & parameter,
  const rclcpp::ParameterValue & value)
{
  auto node = makeNode(suffix);
  node->declare_parameter("FollowPath." + parameter, value);
  bumperbot_motion::MPCController controller;
  try {
    controller.configure(node, "FollowPath", makeBuffer(node), nullptr);
    FAIL() << "configure() accepted invalid parameter " << parameter;
  } catch (const std::invalid_argument & exception) {
    EXPECT_NE(std::string(exception.what()).find("FollowPath"), std::string::npos);
  }
}

}  // namespace

TEST(MPCController, configure_rejects_invalid_parameters_with_clear_error)
{
  initializeRos();
  expectInvalidParameter("horizon", "prediction_horizon", rclcpp::ParameterValue(-1));
  expectInvalidParameter("time_step", "time_step", rclcpp::ParameterValue(0.0));
  expectInvalidParameter(
    "velocity_order", "min_linear_velocity", rclcpp::ParameterValue(0.30));
  expectInvalidParameter(
    "acceleration", "max_linear_acceleration", rclcpp::ParameterValue(0.0));
  expectInvalidParameter("weight", "q_y", rclcpp::ParameterValue(-1.0));
  expectInvalidParameter(
    "wrong_type", "prediction_horizon", rclcpp::ParameterValue(10.0));
}

TEST(MPCController, percentage_absolute_and_no_speed_limit_are_applied_and_reset)
{
  initializeRos();
  auto node = makeNode("speed_limit");
  bumperbot_motion::MPCController controller;
  controller.configure(node, "FollowPath", makeBuffer(node), nullptr);
  controller.activate();
  controller.setPlan(straightPath(node));

  controller.setSpeedLimit(50.0, true);
  const auto percentage_command = controller.computeVelocityCommands(
    robotPose(node), measuredVelocity(0.10), nullptr);
  EXPECT_GE(percentage_command.twist.linear.x, -1e-6);
  EXPECT_LE(percentage_command.twist.linear.x, 0.10 + 1e-6);

  controller.setSpeedLimit(0.06, false);
  const auto absolute_command = controller.computeVelocityCommands(
    robotPose(node), measuredVelocity(0.06), nullptr);
  EXPECT_GE(absolute_command.twist.linear.x, -1e-6);
  EXPECT_LE(absolute_command.twist.linear.x, 0.06 + 1e-6);

  controller.setSpeedLimit(0.0, false);
  const auto reset_command = controller.computeVelocityCommands(
    robotPose(node), measuredVelocity(0.15), nullptr);
  EXPECT_GT(reset_command.twist.linear.x, 0.10);
  EXPECT_LE(reset_command.twist.linear.x, 0.20 + 1e-6);

  controller.setSpeedLimit(25.0, true);
  controller.setSpeedLimit(0.0, true);
  const auto percentage_reset_command = controller.computeVelocityCommands(
    robotPose(node), measuredVelocity(0.15), nullptr);
  EXPECT_GT(percentage_reset_command.twist.linear.x, 0.10);
  EXPECT_LE(percentage_reset_command.twist.linear.x, 0.20 + 1e-6);

  controller.deactivate();
  controller.cleanup();
}

TEST(MPCController, invalid_measured_velocity_returns_finite_zero_command)
{
  initializeRos();
  auto node = makeNode("invalid_velocity");
  bumperbot_motion::MPCController controller;
  controller.configure(node, "FollowPath", makeBuffer(node), nullptr);
  controller.setPlan(straightPath(node));
  auto velocity = measuredVelocity(0.0);
  velocity.angular.z = std::numeric_limits<double>::quiet_NaN();

  const auto command = controller.computeVelocityCommands(robotPose(node), velocity, nullptr);

  EXPECT_TRUE(std::isfinite(command.twist.linear.x));
  EXPECT_TRUE(std::isfinite(command.twist.angular.z));
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
  controller.cleanup();
}

TEST(MPCController, non_monotonic_pose_timestamp_returns_zero_command)
{
  initializeRos();
  auto node = makeNode("timestamp");
  bumperbot_motion::MPCController controller;
  controller.configure(node, "FollowPath", makeBuffer(node), nullptr);
  controller.activate();
  controller.setPlan(straightPath(node));

  auto pose = robotPose(node);
  auto first_command = controller.computeVelocityCommands(pose, measuredVelocity(0.0), nullptr);
  EXPECT_TRUE(std::isfinite(first_command.twist.linear.x));

  pose.header.stamp.sec -= 1;
  const auto invalid_command = controller.computeVelocityCommands(
    pose, measuredVelocity(0.0), nullptr);
  EXPECT_DOUBLE_EQ(invalid_command.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(invalid_command.twist.angular.z, 0.0);
  controller.deactivate();
  controller.cleanup();
}
