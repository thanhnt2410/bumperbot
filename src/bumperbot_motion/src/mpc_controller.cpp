#include "bumperbot_motion/mpc_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bumperbot_motion
{
// Hàm configure được Nav2 gọi một lần khi nạp plugin.
// Hàm này lưu các đối tượng ROS và đọc tham số của MPC từ file YAML.
void MPCController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  plugin_name_ = name;
  if (!node) {
    throw std::invalid_argument("Cannot configure " + plugin_name_ + ": parent node expired");
  }
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();

  // Khai báo giá trị mặc định. Giá trị trong YAML sẽ ghi đè các giá trị này.
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".prediction_horizon", rclcpp::ParameterValue(10));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".time_step", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".reference_linear_velocity", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_distance_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_yaw_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_path_enter_angle", rclcpp::ParameterValue(0.785));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_path_exit_angle", rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_angular_gain", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_position_hysteresis", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_linear_velocity", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_velocity", rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_velocity", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_acceleration", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_acceleration", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".q_x", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".q_y", rclcpp::ParameterValue(20.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".q_yaw", rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".r_v", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".r_omega", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rd_v", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rd_omega", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_cost_weight", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_slack_weight", rclcpp::ParameterValue(100.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_cost_limit", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".side_preference_cost", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".side_preference_activation_cost", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".side_preference_clear_cycles", rclcpp::ParameterValue(10));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".collision_cost_threshold", rclcpp::ParameterValue(253));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".consider_unknown_as_obstacle", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".consider_outside_costmap_as_obstacle", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".collision_failure_cycles", rclcpp::ParameterValue(10));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".collision_failure_timeout", rclcpp::ParameterValue(1.0));

  // Đọc tham số từ Controller Server vào các biến của controller.
  try {
    node->get_parameter(plugin_name_ + ".prediction_horizon", horizon_);
    node->get_parameter(plugin_name_ + ".time_step", time_step_);
    node->get_parameter(plugin_name_ + ".reference_linear_velocity", reference_velocity_);
    node->get_parameter(plugin_name_ + ".goal_distance_tolerance", goal_distance_tolerance_);
    node->get_parameter(plugin_name_ + ".goal_yaw_tolerance", goal_yaw_tolerance_);
    node->get_parameter(plugin_name_ + ".rotate_to_path_enter_angle", rotate_to_path_enter_angle_);
    node->get_parameter(plugin_name_ + ".rotate_to_path_exit_angle", rotate_to_path_exit_angle_);
    node->get_parameter(plugin_name_ + ".goal_angular_gain", goal_angular_gain_);
    node->get_parameter(plugin_name_ + ".goal_position_hysteresis", goal_position_hysteresis_);
    node->get_parameter(plugin_name_ + ".min_linear_velocity", limits_.min_linear_velocity);
    node->get_parameter(plugin_name_ + ".max_linear_velocity", limits_.max_linear_velocity);
    node->get_parameter(plugin_name_ + ".max_angular_velocity", limits_.max_angular_velocity);
    node->get_parameter(
      plugin_name_ + ".max_linear_acceleration", limits_.max_linear_acceleration);
    node->get_parameter(
      plugin_name_ + ".max_angular_acceleration", limits_.max_angular_acceleration);
    node->get_parameter(plugin_name_ + ".q_x", weights_.state[0]);
    node->get_parameter(plugin_name_ + ".q_y", weights_.state[1]);
    node->get_parameter(plugin_name_ + ".q_yaw", weights_.state[2]);
    node->get_parameter(plugin_name_ + ".r_v", weights_.control[0]);
    node->get_parameter(plugin_name_ + ".r_omega", weights_.control[1]);
    node->get_parameter(plugin_name_ + ".rd_v", weights_.control_rate[0]);
    node->get_parameter(plugin_name_ + ".rd_omega", weights_.control_rate[1]);
    node->get_parameter(plugin_name_ + ".obstacle_cost_weight", weights_.obstacle_cost);
    node->get_parameter(
      plugin_name_ + ".obstacle_slack_weight", weights_.obstacle_slack_weight);
    node->get_parameter(plugin_name_ + ".obstacle_cost_limit", limits_.obstacle_cost_limit);
    node->get_parameter(plugin_name_ + ".side_preference_cost", weights_.side_preference_cost);
    node->get_parameter(
      plugin_name_ + ".side_preference_activation_cost",
      weights_.side_preference_activation_cost);
    node->get_parameter(
      plugin_name_ + ".side_preference_clear_cycles", side_preference_clear_cycles_);
    node->get_parameter(plugin_name_ + ".collision_cost_threshold", collision_cost_threshold_);
    node->get_parameter(
      plugin_name_ + ".consider_unknown_as_obstacle", consider_unknown_as_obstacle_);
    node->get_parameter(
      plugin_name_ + ".consider_outside_costmap_as_obstacle",
      consider_outside_costmap_as_obstacle_);
    node->get_parameter(plugin_name_ + ".collision_failure_cycles", collision_failure_cycles_);
    node->get_parameter(plugin_name_ + ".collision_failure_timeout", collision_failure_timeout_);
  } catch (const std::exception & exception) {
    throw std::invalid_argument(
            "Invalid parameter type for " + plugin_name_ + ": " + exception.what());
  }

  // Tăng trọng số tại điểm cuối để robot cố gắng kết thúc gần đường tham chiếu.
  weights_.terminal = 2.0 * weights_.state;
  validateParameters();
  configured_limits_ = limits_;
  reference_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("mpc/reference_path", 10);
  predicted_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("mpc/predicted_path", 10);
  collision_pub_ = node->create_publisher<std_msgs::msg::Bool>("mpc/collision_detected", 10);
  RCLCPP_INFO(logger_, "Configured %s (N=%d, dt=%.3f)", plugin_name_.c_str(), horizon_, time_step_);
}

void MPCController::validateParameters() const
{
  const auto invalid = [this](const std::string & reason) {
      throw std::invalid_argument("Invalid MPC configuration for " + plugin_name_ + ": " + reason);
    };
  if (horizon_ <= 0) {invalid("prediction_horizon must be greater than zero");}
  if (!std::isfinite(time_step_) || time_step_ <= 0.0) {
    invalid("time_step must be finite and greater than zero");
  }
  if (!std::isfinite(reference_velocity_) || reference_velocity_ < 0.0) {
    invalid("reference_linear_velocity must be finite and non-negative");
  }
  if (!std::isfinite(limits_.min_linear_velocity) ||
    !std::isfinite(limits_.max_linear_velocity) ||
    limits_.min_linear_velocity > limits_.max_linear_velocity)
  {
    invalid("min_linear_velocity must not exceed max_linear_velocity");
  }
  if (reference_velocity_ > limits_.max_linear_velocity) {
    invalid("reference_linear_velocity must not exceed max_linear_velocity");
  }
  if (!std::isfinite(limits_.max_angular_velocity) || limits_.max_angular_velocity <= 0.0) {
    invalid("max_angular_velocity must be finite and greater than zero");
  }
  if (!std::isfinite(limits_.max_linear_acceleration) ||
    limits_.max_linear_acceleration <= 0.0)
  {
    invalid("max_linear_acceleration must be finite and greater than zero");
  }
  if (!std::isfinite(limits_.max_angular_acceleration) ||
    limits_.max_angular_acceleration <= 0.0)
  {
    invalid("max_angular_acceleration must be finite and greater than zero");
  }
  if (!weights_.state.allFinite() || !weights_.terminal.allFinite() ||
    !weights_.control.allFinite() || !weights_.control_rate.allFinite() ||
    weights_.state.minCoeff() < 0.0 || weights_.terminal.minCoeff() < 0.0 ||
    weights_.control.minCoeff() < 0.0 || weights_.control_rate.minCoeff() < 0.0 ||
    !std::isfinite(weights_.obstacle_cost) || weights_.obstacle_cost < 0.0)
  {
    invalid("all MPC weights must be finite and non-negative");
  }
  if (!std::isfinite(weights_.obstacle_slack_weight) ||
    weights_.obstacle_slack_weight <= 0.0)
  {
    invalid("obstacle_slack_weight must be finite and positive");
  }
  if (!std::isfinite(weights_.side_preference_cost) || weights_.side_preference_cost < 0.0 ||
    !std::isfinite(weights_.side_preference_activation_cost) ||
    weights_.side_preference_activation_cost < 0.0 ||
    weights_.side_preference_activation_cost > 1.0 ||
    side_preference_clear_cycles_ < 1)
  {
    invalid("side preference cost must be non-negative, threshold in [0, 1], cycles positive");
  }
  if (!std::isfinite(limits_.obstacle_cost_limit) ||
    limits_.obstacle_cost_limit < 0.0 || limits_.obstacle_cost_limit > 1.0)
  {
    invalid("obstacle_cost_limit must be finite and in [0, 1]");
  }
  if (!std::isfinite(goal_distance_tolerance_) || goal_distance_tolerance_ <= 0.0 ||
    !std::isfinite(goal_yaw_tolerance_) || goal_yaw_tolerance_ <= 0.0)
  {
    invalid("goal tolerances must be finite and greater than zero");
  }
  if (!std::isfinite(rotate_to_path_exit_angle_) || rotate_to_path_exit_angle_ < 0.0 ||
    !std::isfinite(rotate_to_path_enter_angle_) ||
    rotate_to_path_enter_angle_ <= rotate_to_path_exit_angle_ ||
    rotate_to_path_enter_angle_ > M_PI)
  {
    invalid("rotate thresholds must satisfy 0 <= exit < enter <= pi");
  }
  if (!std::isfinite(goal_angular_gain_) || goal_angular_gain_ <= 0.0 ||
    !std::isfinite(goal_position_hysteresis_) || goal_position_hysteresis_ < 0.0)
  {
    invalid("goal gain must be positive and goal hysteresis must be non-negative");
  }
  if (collision_cost_threshold_ < 1 || collision_cost_threshold_ > 254) {
    invalid("collision_cost_threshold must be in [1, 254]");
  }
  if (collision_failure_cycles_ < 2 || !std::isfinite(collision_failure_timeout_) ||
    collision_failure_timeout_ <= 0.0)
  {
    invalid("collision failure cycles must be at least 2 and timeout must be positive");
  }
}

void MPCController::activate()
{
  // Plugin bắt đầu nhận yêu cầu tính vận tốc từ Nav2.
  has_last_pose_stamp_ = false;
  consecutive_dead_zone_cycles_ = 0;
  consecutive_side_preference_clear_cycles_ = 0;
  avoidance_direction_ = AvoidanceDirection::NONE;
  resetCollisionState();
  resetSolverMetrics();
  reference_path_pub_->on_activate();
  predicted_path_pub_->on_activate();
  collision_pub_->on_activate();
  RCLCPP_INFO(logger_, "Activating %s", plugin_name_.c_str());
}

void MPCController::deactivate()
{
  // Plugin tạm dừng khi Controller Server chuyển lifecycle state.
  reference_path_pub_->on_deactivate();
  predicted_path_pub_->on_deactivate();
  collision_pub_->on_deactivate();
  RCLCPP_INFO(logger_, "Deactivating %s", plugin_name_.c_str());
}

void MPCController::cleanup()
{
  // Xóa dữ liệu và giải phóng các đối tượng dùng chung.
  RCLCPP_INFO(logger_, "Cleaning up %s", plugin_name_.c_str());
  global_plan_.poses.clear();
  reference_generator_.reset();
  reference_path_pub_.reset();
  predicted_path_pub_.reset();
  collision_pub_.reset();
  solver_.reset();
  tf_.reset();
  costmap_ros_.reset();
  has_last_pose_stamp_ = false;
  consecutive_dead_zone_cycles_ = 0;
  consecutive_side_preference_clear_cycles_ = 0;
  avoidance_direction_ = AvoidanceDirection::NONE;
  resetCollisionState();
  resetSolverMetrics();
}

void MPCController::setPlan(const nav_msgs::msg::Path & path)
{
  // Lưu đường đi mới do planner của Nav2 gửi tới.
  const bool start_new_goal = global_plan_.poses.empty() || control_state_ == ControlState::STOPPED;
  global_plan_ = path;
  reference_generator_.setPlan(path);
  consecutive_dead_zone_cycles_ = 0;
  resetCollisionState();
  if (start_new_goal) {
    consecutive_side_preference_clear_cycles_ = 0;
    avoidance_direction_ = AvoidanceDirection::NONE;
    control_state_ = ControlState::ROTATE_TO_PATH;
  }
}

double MPCController::normalizeAngle(double angle)
{
  // Đưa góc về khoảng [-pi, pi] để tránh sai số góc bị nhảy.
  return std::atan2(std::sin(angle), std::cos(angle));
}

geometry_msgs::msg::TwistStamped MPCController::zeroCommand(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  // Twist mặc định bằng 0, dùng khi không thể tính lệnh điều khiển an toàn.
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = robot_pose.header.stamp;
  command.header.frame_id = robot_pose.header.frame_id;
  return command;
}

geometry_msgs::msg::TwistStamped MPCController::rotationCommand(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & velocity,
  double yaw_error, const MPCLimits & limits)
{
  auto command = zeroCommand(robot_pose);
  const Eigen::Vector2d desired(
    0.0, goal_angular_gain_ * yaw_error);
  const Eigen::Vector2d current(velocity.linear.x, velocity.angular.z);
  const Eigen::Vector2d limited = clampCommand(desired, current, limits);
  command.twist.linear.x = limited[0];
  command.twist.angular.z = limited[1];
  const auto rotation_poses = simulateCommandPoses(robot_pose.pose, command);
  std::vector<ReferencePoint> rotation_reference(rotation_poses.size());
  if (avoidance_direction_ != AvoidanceDirection::NONE) {
    buildAvoidanceReference(robot_pose, rotation_reference);
  } else {
    for (std::size_t index = 0; index < rotation_poses.size(); ++index) {
      rotation_reference[index].pose.pose = rotation_poses[index];
    }
  }
  auto debug_header = robot_pose.header;
  auto node = node_.lock();
  if (node) {
    debug_header.stamp = node->now();
    publishDebugPaths(rotation_reference, rotation_poses, debug_header);
  }
  if (costmap_ros_) {
    const auto costmap_frame = costmap_ros_->getGlobalFrameID();
    if (robot_pose.header.frame_id != costmap_frame) {
      if (node) {
        RCLCPP_WARN_THROTTLE(
          logger_, *node->get_clock(), 2000,
          "Cannot collision-check rotation: pose frame '%s' differs from costmap frame '%s'",
          robot_pose.header.frame_id.c_str(), costmap_frame.c_str());
      }
      return zeroCommand(robot_pose);
    }
    if (!isPredictedTrajectoryCollisionFree(rotation_poses)) {
      updateCollisionState(true);
      if (node) {
        RCLCPP_WARN_THROTTLE(
          logger_, *node->get_clock(), 1000,
          "MPC predicted footprint collision while rotating; commanding zero velocity");
      }
      return zeroCommand(robot_pose);
    }
    updateCollisionState(false);
  }
  return command;
}

Eigen::Vector2d MPCController::clampCommand(
  const Eigen::Vector2d & desired, const Eigen::Vector2d & current,
  const MPCLimits & limits) const
{
  const auto clamp_axis = [](double value, double measured, double minimum, double maximum,
      double maximum_change) {
      const double lower = std::max(minimum, measured - maximum_change);
      const double upper = std::min(maximum, measured + maximum_change);
      // Nếu measured velocity đã nằm ngoài hard bounds thì không tồn tại command vừa thỏa
      // velocity bound vừa thỏa rate bound. Ưu tiên hard velocity bound trong trường hợp này.
      return lower <= upper ? std::clamp(value, lower, upper) :
             std::clamp(value, minimum, maximum);
    };
  return Eigen::Vector2d(
    clamp_axis(
      desired[0], current[0], limits.min_linear_velocity, limits.max_linear_velocity,
      limits.max_linear_acceleration * time_step_),
    clamp_axis(
      desired[1], current[1], -limits.max_angular_velocity, limits.max_angular_velocity,
      limits.max_angular_acceleration * time_step_));
}

bool MPCController::runtimeInputValid(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & velocity) const
{
  const auto & position = robot_pose.pose.position;
  const auto & orientation = robot_pose.pose.orientation;
  const double quaternion_norm =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  if (robot_pose.header.frame_id.empty() || robot_pose.header.stamp.sec < 0 ||
    robot_pose.header.stamp.nanosec >= 1000000000U ||
    !std::isfinite(position.x) || !std::isfinite(position.y) ||
    !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
    !std::isfinite(orientation.z) || !std::isfinite(orientation.w) ||
    !std::isfinite(quaternion_norm) || quaternion_norm <= 1e-12 ||
    !std::isfinite(velocity.linear.x) || !std::isfinite(velocity.angular.z))
  {
    return false;
  }

  // A zero stamp is commonly used when simulation time is not active. For
  // non-zero stamps, reject time going backwards or repeating: the MPC
  // acceleration constraint must be based on a valid controller cycle.
  const rclcpp::Time stamp(robot_pose.header.stamp, RCL_ROS_TIME);
  if (stamp.nanoseconds() != 0) {
    if (has_last_pose_stamp_ && stamp <= last_pose_stamp_) {
      return false;
    }
    last_pose_stamp_ = stamp;
    has_last_pose_stamp_ = true;
  }
  return true;
}

bool MPCController::isFootprintCollisionFreeLocked(
  const geometry_msgs::msg::Pose & pose, nav2_costmap_2d::Costmap2D * costmap,
  const nav2_costmap_2d::Footprint & footprint) const
{
  if (costmap == nullptr || footprint.empty()) {
    return false;
  }

  const double yaw = tf2::getYaw(pose.orientation);
  unsigned int map_x = 0;
  unsigned int map_y = 0;
  bool outside = !costmap->worldToMap(pose.position.x, pose.position.y, map_x, map_y);
  for (const auto & point : footprint) {
    const double world_x = pose.position.x + std::cos(yaw) * point.x - std::sin(yaw) * point.y;
    const double world_y = pose.position.y + std::sin(yaw) * point.x + std::cos(yaw) * point.y;
    outside = outside || !costmap->worldToMap(world_x, world_y, map_x, map_y);
  }
  if (outside) {return !consider_outside_costmap_as_obstacle_;}

  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);
  const double cost = checker.footprintCostAtPose(
    pose.position.x, pose.position.y, yaw, footprint);

  if (!std::isfinite(cost)) {return false;}
  if (cost == nav2_costmap_2d::NO_INFORMATION) {return !consider_unknown_as_obstacle_;}
  return cost >= 0.0 && cost < collision_cost_threshold_;
}

bool MPCController::isPredictedTrajectoryCollisionFree(
  const std::vector<geometry_msgs::msg::Pose> & poses) const
{
  if (poses.empty() || !costmap_ros_) {return false;}
  auto * costmap = costmap_ros_->getCostmap();
  const auto footprint = costmap_ros_->getRobotFootprint();
  if (costmap == nullptr || footprint.empty()) {return false;}
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
  return isPredictedTrajectoryCollisionFreeLocked(poses, costmap, footprint);
}

// Kiểm tra corridor vừa không va chạm vừa nằm ngoài vùng cost MPC cần tránh.
bool MPCController::isObstacleCostCorridorClear(
  const std::vector<geometry_msgs::msg::Pose> & poses,
  double maximum_normalized_cost) const
{
  if (poses.empty() || !costmap_ros_) {return false;}
  auto * costmap = costmap_ros_->getCostmap();
  const auto footprint = costmap_ros_->getRobotFootprint();
  if (costmap == nullptr || footprint.empty()) {return false;}
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
  return isObstacleCostCorridorClearLocked(
    poses, costmap, footprint, maximum_normalized_cost);
}

// Dùng cùng ngưỡng cost với solver để tránh chuyển pha khi còn trong inflation.
bool MPCController::isObstacleCostCorridorClearLocked(
  const std::vector<geometry_msgs::msg::Pose> & poses,
  nav2_costmap_2d::Costmap2D * costmap,
  const nav2_costmap_2d::Footprint & footprint,
  double maximum_normalized_cost) const
{
  if (!isPredictedTrajectoryCollisionFreeLocked(poses, costmap, footprint)) {return false;}
  return std::all_of(
    poses.begin(), poses.end(),
    [&](const geometry_msgs::msg::Pose & pose) {
      const double normalized_cost = normalizedCostAtWorldLocked(
        pose.position.x, pose.position.y, costmap);
      return normalized_cost <= maximum_normalized_cost;
    });
}

bool MPCController::isPredictedTrajectoryCollisionFreeLocked(
  const std::vector<geometry_msgs::msg::Pose> & poses,
  nav2_costmap_2d::Costmap2D * costmap,
  const nav2_costmap_2d::Footprint & footprint) const
{
  if (poses.empty() || costmap == nullptr || footprint.empty()) {return false;}
  const double linear_step = 0.5 * costmap->getResolution();
  constexpr double angular_step = 0.1;
  if (!std::isfinite(linear_step) || linear_step <= 0.0) {return false;}
  if (!isFootprintCollisionFreeLocked(poses.front(), costmap, footprint)) {return false;}

  for (std::size_t segment = 0; segment + 1 < poses.size(); ++segment) {
    const auto & start = poses[segment];
    const auto & finish = poses[segment + 1];
    const double dx = finish.position.x - start.position.x;
    const double dy = finish.position.y - start.position.y;
    const double start_yaw = tf2::getYaw(start.orientation);
    const double yaw_delta = normalizeAngle(tf2::getYaw(finish.orientation) - start_yaw);
    const double distance = std::hypot(dx, dy);
    if (!std::isfinite(distance) || !std::isfinite(yaw_delta)) {return false;}

    const auto linear_samples = static_cast<std::size_t>(std::ceil(distance / linear_step));
    const auto angular_samples = static_cast<std::size_t>(
      std::ceil(std::abs(yaw_delta) / angular_step));
    const std::size_t samples = std::max<std::size_t>(
      1, std::max(linear_samples, angular_samples));

    for (std::size_t sample = 1; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
      geometry_msgs::msg::Pose interpolated;
      interpolated.position.x = start.position.x + ratio * dx;
      interpolated.position.y = start.position.y + ratio * dy;
      const double yaw = start_yaw + ratio * yaw_delta;
      interpolated.orientation.z = std::sin(0.5 * yaw);
      interpolated.orientation.w = std::cos(0.5 * yaw);
      if (!isFootprintCollisionFreeLocked(interpolated, costmap, footprint)) {return false;}
    }
  }
  return true;
}

std::vector<MPCObstacleSample> MPCController::sampleObstacleCostsLocked(
  const std::vector<geometry_msgs::msg::Pose> & poses,
  nav2_costmap_2d::Costmap2D * costmap, double obstacle_cost_limit) const
{
  std::vector<MPCObstacleSample> samples;
  if (costmap == nullptr) {return samples;}

  const double resolution = costmap->getResolution();
  if (!std::isfinite(resolution) || resolution <= 0.0) {return samples;}

  samples.reserve(poses.size());
  for (const auto & pose : poses) {
    const double x = pose.position.x;
    const double y = pose.position.y;
    MPCObstacleSample sample;
    sample.cost = normalizedCostAtWorldLocked(x, y, costmap);
    sample.linearization_yaw = tf2::getYaw(pose.orientation);
    sample.world_gradient.x() =
      (normalizedCostAtWorldLocked(x + resolution, y, costmap) -
      normalizedCostAtWorldLocked(x - resolution, y, costmap)) /
      (2.0 * resolution);
    sample.world_gradient.y() =
      (normalizedCostAtWorldLocked(x, y + resolution, costmap) -
      normalizedCostAtWorldLocked(x, y - resolution, costmap)) /
      (2.0 * resolution);
    if (sample.cost > obstacle_cost_limit && sample.world_gradient.norm() < 1e-6) {
      sample.world_gradient = calculateFallbackObstacleGradientLocked(
        pose, costmap, 3.0 * resolution);
    }
    samples.push_back(sample);
  }
  return samples;
}

double MPCController::normalizedCostAtWorldLocked(
  double world_x, double world_y, nav2_costmap_2d::Costmap2D * costmap) const
{
  unsigned int map_x = 0;
  unsigned int map_y = 0;
  if (!costmap->worldToMap(world_x, world_y, map_x, map_y)) {
    return consider_outside_costmap_as_obstacle_ ? 1.0 : 0.0;
  }
  const unsigned char raw_cost = costmap->getCost(map_x, map_y);
  if (raw_cost == nav2_costmap_2d::NO_INFORMATION) {
    return consider_unknown_as_obstacle_ ? 1.0 : 0.0;
  }
  return std::clamp(
    static_cast<double>(raw_cost) /
    static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE), 0.0, 1.0);
}

Eigen::Vector2d MPCController::calculateFallbackObstacleGradientLocked(
  const geometry_msgs::msg::Pose & pose,
  nav2_costmap_2d::Costmap2D * costmap, double probe_distance) const
{
  const Eigen::Vector2d lateral_costs =
    sampleLateralCostsLocked(pose, costmap, probe_distance);
  const double left_cost = lateral_costs.x();
  const double right_cost = lateral_costs.y();
  const double yaw = tf2::getYaw(pose.orientation);
  const Eigen::Vector2d lateral_direction(-std::sin(yaw), std::cos(yaw));
  const double lateral_cost_difference = left_cost - right_cost;
  if (std::abs(lateral_cost_difference) <= 1e-3) {
    return Eigen::Vector2d::Zero();
  }
  return lateral_cost_difference / (2.0 * probe_distance) * lateral_direction;
}

// Đo cost tại hai điểm đối xứng theo phương ngang của pose.
Eigen::Vector2d MPCController::sampleLateralCostsLocked(
  const geometry_msgs::msg::Pose & pose,
  nav2_costmap_2d::Costmap2D * costmap, double probe_distance) const
{
  const double world_yaw = tf2::getYaw(pose.orientation);
  const Eigen::Vector2d world_lateral_direction(-std::sin(world_yaw), std::cos(world_yaw));
  const Eigen::Vector2d position(pose.position.x, pose.position.y);
  const Eigen::Vector2d left_position = position + probe_distance * world_lateral_direction;
  const Eigen::Vector2d right_position = position - probe_distance * world_lateral_direction;
  const double left_cost = normalizedCostAtWorldLocked(
    left_position.x(), left_position.y(), costmap);
  const double right_cost = normalizedCostAtWorldLocked(
    right_position.x(), right_position.y(), costmap);
  return Eigen::Vector2d(left_cost, right_cost);
}

// Đếm số chu kỳ đứng yên liên tiếp khi goal xa và reference bị vật cản chặn.
bool MPCController::updateDeadZoneState(
  bool linear_motion_is_low, bool goal_is_far, bool obstacle_is_ahead)
{
  if (!linear_motion_is_low || !goal_is_far || !obstacle_is_ahead) {
    consecutive_dead_zone_cycles_ = 0;
    return false;
  }
  ++consecutive_dead_zone_cycles_;
  return consecutive_dead_zone_cycles_ >= dead_zone_detection_cycles_;
}

// Chọn phía có cost thấp hơn một lần và giữ quyết định tới khi avoidance kết thúc.
void MPCController::lockAvoidanceDirection(const Eigen::Vector2d & lateral_costs)
{
  if (avoidance_direction_ != AvoidanceDirection::NONE) {
    return;
  }
  const double left_cost = lateral_costs.x();
  const double right_cost = lateral_costs.y();
  avoidance_direction_ =
    left_cost <= right_cost ? AvoidanceDirection::LEFT : AvoidanceDirection::RIGHT;
}

// Khóa phía khi có obstacle và chỉ mở sau nhiều chu kỳ cost thấp liên tiếp.
void MPCController::updateSidePreference(
  bool obstacle_is_ahead, const Eigen::Vector2d & lateral_costs)
{
  const bool structured_avoidance_is_active =
    control_state_ == ControlState::AVOID_ROTATE ||
    control_state_ == ControlState::AVOID_TRACKING;
  if (structured_avoidance_is_active) {
    return;
  }
  if (obstacle_is_ahead) {
    consecutive_side_preference_clear_cycles_ = 0;
    lockAvoidanceDirection(lateral_costs);
    return;
  }
  if (avoidance_direction_ == AvoidanceDirection::NONE) {
    consecutive_side_preference_clear_cycles_ = 0;
    return;
  }
  ++consecutive_side_preference_clear_cycles_;
  if (consecutive_side_preference_clear_cycles_ >= side_preference_clear_cycles_) {
    consecutive_side_preference_clear_cycles_ = 0;
    avoidance_direction_ = AvoidanceDirection::NONE;
  }
}

// Lưu pose bắt đầu và đặt yaw mục tiêu về phía tránh đã khóa.
void MPCController::startAvoidanceRotation(
  const geometry_msgs::msg::PoseStamped & robot_pose, double path_world_yaw)
{
  if (avoidance_direction_ == AvoidanceDirection::NONE) {
    return;
  }
  const double direction_sign = static_cast<int>(avoidance_direction_);
  const double avoidance_yaw_offset = direction_sign * avoidance_rotation_angle_;
  avoidance_path_world_yaw_ = path_world_yaw;
  avoidance_path_world_position_ = Eigen::Vector2d(
    robot_pose.pose.position.x, robot_pose.pose.position.y);
  avoidance_target_world_yaw_ =
    normalizeAngle(avoidance_path_world_yaw_ + avoidance_yaw_offset);
  avoidance_start_world_position_ = Eigen::Vector2d(
    robot_pose.pose.position.x, robot_pose.pose.position.y);
  avoidance_phase_ = AvoidancePhase::CLEAR_LATERAL;
  control_state_ = ControlState::AVOID_ROTATE;
}

// Tạo reference cục bộ chéo theo yaw tránh để robot thực sự đi sang bên đã chọn.
void MPCController::buildAvoidanceReference(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  std::vector<ReferencePoint> & reference) const
{
  if (avoidance_direction_ == AvoidanceDirection::NONE) {
    return;
  }
  const Eigen::Vector2d world_avoidance_direction(
    std::cos(avoidance_target_world_yaw_), std::sin(avoidance_target_world_yaw_));
  const Eigen::Vector2d start_world_position(
    robot_pose.pose.position.x, robot_pose.pose.position.y);
  double reference_travel_distance = 0.0;
  for (auto & point : reference) {
    const Eigen::Vector2d reference_world_position =
      start_world_position + reference_travel_distance * world_avoidance_direction;
    point.pose.pose.position.x = reference_world_position.x();
    point.pose.pose.position.y = reference_world_position.y();
    point.pose.pose.orientation.z = std::sin(0.5 * avoidance_target_world_yaw_);
    point.pose.pose.orientation.w = std::cos(0.5 * avoidance_target_world_yaw_);
    point.linear_velocity = avoidance_reference_velocity_;
    point.angular_velocity = 0.0;
    point.curvature = 0.0;
    point.arc_length = reference_travel_distance;
    reference_travel_distance += avoidance_reference_velocity_ * time_step_;
  }
}

// Kiểm tra robot đã đi đủ quãng đường để vượt qua vùng vật cản hay chưa.
bool MPCController::avoidanceTravelComplete(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  const Eigen::Vector2d current_world_position(
    robot_pose.pose.position.x, robot_pose.pose.position.y);
  const Eigen::Vector2d phase_world_direction(
    std::cos(avoidance_target_world_yaw_), std::sin(avoidance_target_world_yaw_));
  const Eigen::Vector2d phase_world_displacement =
    current_world_position - avoidance_start_world_position_;
  const double avoidance_travel_distance =
    phase_world_displacement.dot(phase_world_direction);
  double required_travel_distance = avoidance_rejoin_distance_;
  if (avoidance_phase_ == AvoidancePhase::CLEAR_LATERAL) {
    required_travel_distance = avoidance_lateral_clearance_;
  } else if (avoidance_phase_ == AvoidancePhase::PASS_OBSTACLE) {
    required_travel_distance = avoidance_pass_distance_;
  }
  return avoidance_travel_distance >= required_travel_distance;
}

// Tạo corridor ngắn từ vị trí tránh hiện tại về đường global ban đầu.
std::vector<geometry_msgs::msg::Pose> MPCController::buildAvoidanceRejoinPoses(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  constexpr int rejoin_samples = 10;
  const Eigen::Vector2d path_world_direction(
    std::cos(avoidance_path_world_yaw_), std::sin(avoidance_path_world_yaw_));
  const Eigen::Vector2d current_world_position(
    robot_pose.pose.position.x, robot_pose.pose.position.y);
  const Eigen::Vector2d path_world_displacement =
    current_world_position - avoidance_path_world_position_;
  const double path_longitudinal_position =
    path_world_displacement.dot(path_world_direction);
  const Eigen::Vector2d rejoin_world_position =
    avoidance_path_world_position_ + path_longitudinal_position * path_world_direction;

  std::vector<geometry_msgs::msg::Pose> poses;
  poses.reserve(rejoin_samples + 1);
  for (int sample = 0; sample <= rejoin_samples; ++sample) {
    const double ratio = static_cast<double>(sample) / rejoin_samples;
    const Eigen::Vector2d sampled_world_position =
      current_world_position + ratio * (rejoin_world_position - current_world_position);
    auto pose = robot_pose.pose;
    pose.position.x = sampled_world_position.x();
    pose.position.y = sampled_world_position.y();
    pose.orientation.z = std::sin(0.5 * avoidance_path_world_yaw_);
    pose.orientation.w = std::cos(0.5 * avoidance_path_world_yaw_);
    poses.push_back(pose);
  }
  return poses;
}

// Tạo corridor theo hướng global để kiểm tra đã lệch đủ xa khỏi vật cản chưa.
std::vector<geometry_msgs::msg::Pose> MPCController::buildAvoidanceForwardPoses(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  constexpr int forward_samples = 20;
  const Eigen::Vector2d path_world_direction(
    std::cos(avoidance_path_world_yaw_), std::sin(avoidance_path_world_yaw_));
  const Eigen::Vector2d current_world_position(
    robot_pose.pose.position.x, robot_pose.pose.position.y);
  std::vector<geometry_msgs::msg::Pose> poses;
  poses.reserve(forward_samples + 1);
  for (int sample = 0; sample <= forward_samples; ++sample) {
    const double ratio = static_cast<double>(sample) / forward_samples;
    const Eigen::Vector2d sampled_world_position =
      current_world_position + ratio * avoidance_pass_distance_ * path_world_direction;
    auto pose = robot_pose.pose;
    pose.position.x = sampled_world_position.x();
    pose.position.y = sampled_world_position.y();
    pose.orientation.z = std::sin(0.5 * avoidance_path_world_yaw_);
    pose.orientation.w = std::cos(0.5 * avoidance_path_world_yaw_);
    poses.push_back(pose);
  }
  return poses;
}

void MPCController::publishCollisionState(bool collision) const
{
  if (collision_pub_) {
    std_msgs::msg::Bool message;
    message.data = collision;
    collision_pub_->publish(message);
  }
}

void MPCController::resetCollisionState()
{
  consecutive_collision_cycles_ = 0;
  collision_timer_active_ = false;
}

void MPCController::updateCollisionState(bool collision)
{
  publishCollisionState(collision);
  if (!collision) {resetCollisionState(); return;}

  const auto now = std::chrono::steady_clock::now();
  if (!collision_timer_active_) {
    collision_start_time_ = now;
    collision_timer_active_ = true;
  }
  ++consecutive_collision_cycles_;
  const double elapsed = std::chrono::duration<double>(now - collision_start_time_).count();
  if (consecutive_collision_cycles_ >= collision_failure_cycles_ ||
    elapsed >= collision_failure_timeout_)
  {
    throw nav2_core::PlannerException(
            "MPC collision persisted; requesting replanning or recovery");
  }
}

void MPCController::resetSolverMetrics()
{
  solve_time_samples_.clear();
  callback_time_samples_.clear();
  iteration_samples_.clear();
  solver_sample_count_ = 0;
  solver_failure_count_ = 0;
}

void MPCController::recordSolverMetrics(
  const MPCResult & result,
  const std::chrono::steady_clock::time_point & compute_start)
{
  constexpr std::size_t window_size = 100;
  const double callback_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - compute_start).count();
  solve_time_samples_.push_back(result.solve_time_ms);
  callback_time_samples_.push_back(callback_ms);
  iteration_samples_.push_back(result.iterations);
  ++solver_sample_count_;
  if (!result.solved) {++solver_failure_count_;}

  if (solver_sample_count_ == 1) {
    RCLCPP_INFO(
      logger_, "MPC baseline first sample: status=%s iter=%d solve=%.3f ms callback=%.3f ms",
      result.status.c_str(), result.iterations, result.solve_time_ms, callback_ms);
  }
  if (solve_time_samples_.size() < window_size) {return;}

  const auto metrics = [](const auto & samples) {
      using Value = typename std::decay_t<decltype(samples)>::value_type;
      std::vector<Value> sorted(samples);
      std::sort(sorted.begin(), sorted.end());
      const std::size_t p95_index = (95 * sorted.size() + 99) / 100 - 1;
      const double mean = std::accumulate(sorted.begin(), sorted.end(), 0.0) /
        static_cast<double>(sorted.size());
      return std::array<double, 3>{
      mean, static_cast<double>(sorted[p95_index]), static_cast<double>(sorted.back())};
    };
  const auto solve = metrics(solve_time_samples_);
  const auto callback = metrics(callback_time_samples_);
  const auto iterations = metrics(iteration_samples_);
  RCLCPP_INFO(
    logger_, "MPC baseline N=%d samples=%zu status=%s failures=%zu "
    "solve_ms(mean/p95/max)=%.3f/%.3f/%.3f callback_ms=%.3f/%.3f/%.3f "
    "iterations=%.1f/%.0f/%.0f",
    horizon_, window_size, result.status.c_str(), solver_failure_count_,
    solve[0], solve[1], solve[2], callback[0], callback[1], callback[2],
    iterations[0], iterations[1], iterations[2]);
  solve_time_samples_.clear();
  callback_time_samples_.clear();
  iteration_samples_.clear();
  solver_failure_count_ = 0;
}

std::vector<geometry_msgs::msg::Pose> MPCController::buildPredictedPoses(
  const std::vector<ReferencePoint> & reference, const MPCResult & result) const
{
  std::vector<geometry_msgs::msg::Pose> poses;
  poses.reserve(result.predicted_errors.size());
  for (std::size_t k = 0; k < result.predicted_errors.size() && k < reference.size(); ++k) {
    const auto & ref_pose = reference[k].pose.pose;
    const auto & error = result.predicted_errors[k];
    const double yaw = normalizeAngle(tf2::getYaw(ref_pose.orientation) - error[2]);
    geometry_msgs::msg::Pose pose;
    pose.position.x = ref_pose.position.x -
      std::cos(yaw) * error[0] + std::sin(yaw) * error[1];
    pose.position.y = ref_pose.position.y -
      std::sin(yaw) * error[0] - std::cos(yaw) * error[1];
    pose.orientation.z = std::sin(0.5 * yaw);
    pose.orientation.w = std::cos(0.5 * yaw);
    poses.push_back(pose);
  }
  return poses;
}

std::vector<geometry_msgs::msg::Pose> MPCController::simulateCommandPoses(
  const geometry_msgs::msg::Pose & start_pose,
  const geometry_msgs::msg::TwistStamped & command) const
{
  std::vector<geometry_msgs::msg::Pose> poses;
  poses.reserve(static_cast<std::size_t>(horizon_ + 1));
  auto pose = start_pose;
  double yaw = tf2::getYaw(pose.orientation);
  poses.push_back(pose);

  for (int step = 0; step < horizon_; ++step) {
    pose.position.x += time_step_ * command.twist.linear.x * std::cos(yaw);
    pose.position.y += time_step_ * command.twist.linear.x * std::sin(yaw);
    yaw = normalizeAngle(yaw + time_step_ * command.twist.angular.z);
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(0.5 * yaw);
    pose.orientation.w = std::cos(0.5 * yaw);
    poses.push_back(pose);
  }
  return poses;
}

void MPCController::publishDebugPaths(
  const std::vector<ReferencePoint> & reference,
  const std::vector<geometry_msgs::msg::Pose> & predicted_poses,
  const std_msgs::msg::Header & header) const
{
  nav_msgs::msg::Path reference_path;
  reference_path.header = header;
  reference_path.poses.reserve(reference.size());
  for (const auto & point : reference) {
    auto pose = point.pose;
    pose.header = header;
    reference_path.poses.push_back(pose);
  }
  reference_path_pub_->publish(reference_path);

  nav_msgs::msg::Path predicted_path;
  predicted_path.header = header;
  predicted_path.poses.reserve(predicted_poses.size());
  for (const auto & pose : predicted_poses) {
    geometry_msgs::msg::PoseStamped predicted_pose;
    predicted_pose.header = header;
    predicted_pose.pose = pose;
    predicted_path.poses.push_back(predicted_pose);
  }
  predicted_path_pub_->publish(predicted_path);
}

bool MPCController::transformPlan(
  const std::string & target_frame,
  nav_msgs::msg::Path & transformed) const
{
  // Chỉ biến đổi bản sao, không thay đổi global_plan_ gốc.
  transformed = global_plan_;
  if (transformed.header.frame_id.empty()) {return false;}
  if (transformed.header.frame_id == target_frame) {return true;}
  try {
    // Tìm phép biến đổi TF từ frame của path sang frame của robot.
    const auto transform = tf_->lookupTransform(
      target_frame, transformed.header.frame_id,
      tf2::TimePointZero);
    for (auto & pose : transformed.poses) {
      tf2::doTransform(pose, pose, transform);
    }
    transformed.header.frame_id = target_frame;
    return true;
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN(
      logger_, "Cannot transform MPC plan from %s to %s: %s",
      transformed.header.frame_id.c_str(), target_frame.c_str(), exception.what());
    return false;
  }
}

geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const auto compute_start = std::chrono::steady_clock::now();
  // Luôn chuẩn bị lệnh dừng để trả về nếu một bước bên dưới thất bại.
  auto command = zeroCommand(robot_pose);
  auto node = node_.lock();
  if (!node) {
    return command;
  }
  if (!runtimeInputValid(robot_pose, velocity)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 2000,
      "MPC received invalid pose timestamp, pose, or measured velocity; commanding zero");
    return command;
  }
  MPCLimits active_limits;
  {
    std::lock_guard<std::mutex> lock(limits_mutex_);
    active_limits = limits_;
  }

  // Không có path thì robot phải dừng.
  if (global_plan_.poses.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 2000, "MPC received an empty plan");
    return command;
  }
  nav_msgs::msg::Path transformed;
  if (!transformPlan(robot_pose.header.frame_id, transformed)) {return command;}

  double xy_tolerance = goal_distance_tolerance_;
  double yaw_tolerance = goal_yaw_tolerance_;
  if (goal_checker != nullptr) {
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    if (goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
      if (std::isfinite(pose_tolerance.position.x) && pose_tolerance.position.x > 0.0) {
        xy_tolerance = pose_tolerance.position.x;
      }
      if (std::isfinite(pose_tolerance.orientation.z) && pose_tolerance.orientation.z > 0.0) {
        yaw_tolerance = pose_tolerance.orientation.z;
      }
    }
  }
  const auto & goal = transformed.poses.back().pose.position;
  const double goal_distance = std::hypot(
    goal.x - robot_pose.pose.position.x,
    goal.y - robot_pose.pose.position.y);
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double goal_yaw_error = normalizeAngle(
    tf2::getYaw(transformed.poses.back().pose.orientation) - robot_yaw);

  if (control_state_ == ControlState::STOPPED &&
    goal_distance <= xy_tolerance + goal_position_hysteresis_)
  {
    return command;
  }
  if (control_state_ == ControlState::GOAL_ALIGN &&
    goal_distance <= xy_tolerance + goal_position_hysteresis_)
  {
    if (std::abs(goal_yaw_error) <= yaw_tolerance) {
      control_state_ = ControlState::STOPPED;
      return command;
    }
    return rotationCommand(robot_pose, velocity, goal_yaw_error, active_limits);
  }
  if (goal_distance <= xy_tolerance) {
    if (std::abs(goal_yaw_error) <= yaw_tolerance) {
      control_state_ = ControlState::STOPPED;
      return command;
    }
    control_state_ = ControlState::GOAL_ALIGN;
    return rotationCommand(robot_pose, velocity, goal_yaw_error, active_limits);
  }
  if (control_state_ == ControlState::GOAL_ALIGN || control_state_ == ControlState::STOPPED) {
    control_state_ = ControlState::MPC_TRACKING;
  }
  if (control_state_ == ControlState::AVOID_ROTATE) {
    const double avoidance_yaw_error =
      normalizeAngle(avoidance_target_world_yaw_ - robot_yaw);
    if (std::abs(avoidance_yaw_error) > avoidance_yaw_tolerance_) {
      return rotationCommand(robot_pose, velocity, avoidance_yaw_error, active_limits);
    }
    control_state_ = ControlState::AVOID_TRACKING;
  }
  if (control_state_ == ControlState::AVOID_TRACKING &&
    avoidanceTravelComplete(robot_pose))
  {
    if (avoidance_phase_ == AvoidancePhase::CLEAR_LATERAL) {
      const bool forward_corridor_is_clear = !costmap_ros_ ||
        isObstacleCostCorridorClear(
        buildAvoidanceForwardPoses(robot_pose), active_limits.obstacle_cost_limit);
      if (forward_corridor_is_clear) {
        avoidance_phase_ = AvoidancePhase::PASS_OBSTACLE;
        avoidance_target_world_yaw_ = avoidance_path_world_yaw_;
        avoidance_start_world_position_ = Eigen::Vector2d(
          robot_pose.pose.position.x, robot_pose.pose.position.y);
        control_state_ = ControlState::AVOID_ROTATE;
        const double path_yaw_error =
          normalizeAngle(avoidance_target_world_yaw_ - robot_yaw);
        RCLCPP_INFO(logger_, "MPC lateral corridor clear; rotating parallel to path");
        return rotationCommand(robot_pose, velocity, path_yaw_error, active_limits);
      }
      RCLCPP_INFO_THROTTLE(
        logger_, *node->get_clock(), 2000,
        "MPC forward corridor is blocked; continuing lateral avoidance");
    } else if (avoidance_phase_ == AvoidancePhase::PASS_OBSTACLE) {
      const auto rejoin_poses = buildAvoidanceRejoinPoses(robot_pose);
      const bool rejoin_corridor_is_clear = !costmap_ros_ ||
        isObstacleCostCorridorClear(rejoin_poses, active_limits.obstacle_cost_limit);
      if (rejoin_corridor_is_clear) {
        const Eigen::Vector2d current_world_position(
          robot_pose.pose.position.x, robot_pose.pose.position.y);
        const Eigen::Vector2d rejoin_world_position(
          rejoin_poses.back().position.x, rejoin_poses.back().position.y);
        const Eigen::Vector2d rejoin_world_displacement =
          rejoin_world_position - current_world_position;
        avoidance_rejoin_distance_ = rejoin_world_displacement.norm();
        avoidance_target_world_yaw_ = std::atan2(
          rejoin_world_displacement.y(), rejoin_world_displacement.x());
        avoidance_start_world_position_ = current_world_position;
        avoidance_phase_ = AvoidancePhase::REJOIN_PATH;
        control_state_ = ControlState::AVOID_ROTATE;
        const double rejoin_yaw_error =
          normalizeAngle(avoidance_target_world_yaw_ - robot_yaw);
        RCLCPP_INFO(logger_, "MPC pass complete; rotating toward safe rejoin path");
        return rotationCommand(robot_pose, velocity, rejoin_yaw_error, active_limits);
      }
      RCLCPP_INFO_THROTTLE(
        logger_, *node->get_clock(), 2000,
        "MPC rejoin corridor is blocked; continuing parallel avoidance");
    } else {
      control_state_ = ControlState::MPC_TRACKING;
      consecutive_side_preference_clear_cycles_ = 0;
      avoidance_direction_ = AvoidanceDirection::NONE;
      consecutive_dead_zone_cycles_ = 0;
      RCLCPP_INFO(logger_, "MPC safe rejoin complete; resuming global reference");
    }
  }

  // Tạo N + 1 điểm tham chiếu phía trước robot.
  auto reference = reference_generator_.generate(
    transformed, robot_pose, horizon_, time_step_, reference_velocity_,
    velocity.linear.x, active_limits.max_linear_acceleration,
    active_limits.max_linear_velocity, active_limits.max_angular_velocity);
  if (reference.size() != static_cast<std::size_t>(horizon_ + 1)) {
    RCLCPP_WARN(logger_, "MPC could not generate a reference trajectory");
    return command;
  }
  if (control_state_ == ControlState::AVOID_TRACKING) {
    buildAvoidanceReference(robot_pose, reference);
  }
  // Tính sai số vị trí và góc trong hệ tọa độ gắn với robot.
  const double dx = reference.front().pose.pose.position.x - robot_pose.pose.position.x;
  const double dy = reference.front().pose.pose.position.y - robot_pose.pose.position.y;
  Eigen::Vector3d error(
    std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy,
    -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy,
    normalizeAngle(tf2::getYaw(reference.front().pose.pose.orientation) - robot_yaw));
  if (control_state_ == ControlState::ROTATE_TO_PATH) {
    if (std::abs(error[2]) > rotate_to_path_exit_angle_) {
      return rotationCommand(robot_pose, velocity, error[2], active_limits);
    }
    control_state_ = ControlState::MPC_TRACKING;
  } else if (control_state_ == ControlState::MPC_TRACKING &&
    std::abs(error[2]) >= rotate_to_path_enter_angle_)
  {
    control_state_ = ControlState::ROTATE_TO_PATH;
    return rotationCommand(robot_pose, velocity, error[2], active_limits);
  }
  std::vector<MPCObstacleSample> obstacle_samples;
  std::size_t obstacle_probe_index = 0U;
  double highest_obstacle_cost = 0.0;
  Eigen::Vector2d lateral_obstacle_costs = Eigen::Vector2d::Zero();
  if (costmap_ros_ && robot_pose.header.frame_id == costmap_ros_->getGlobalFrameID()) {
    auto * costmap = costmap_ros_->getCostmap();
    if (costmap != nullptr) {
      std::vector<geometry_msgs::msg::Pose> linearization_poses;
      linearization_poses.reserve(reference.size());
      for (const auto & point : reference) {
        linearization_poses.push_back(point.pose.pose);
      }
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
      obstacle_samples = sampleObstacleCostsLocked(
        linearization_poses, costmap, active_limits.obstacle_cost_limit);
      const auto highest_cost_sample = std::max_element(
        obstacle_samples.begin(), obstacle_samples.end(),
        [](const auto & lhs, const auto & rhs) {return lhs.cost < rhs.cost;});
      if (highest_cost_sample != obstacle_samples.end()) {
        obstacle_probe_index = static_cast<std::size_t>(
          std::distance(obstacle_samples.begin(), highest_cost_sample));
        highest_obstacle_cost = highest_cost_sample->cost;
      }
      const double lateral_probe_activation_cost = std::min(
        weights_.side_preference_activation_cost, active_limits.obstacle_cost_limit);
      if (highest_obstacle_cost > lateral_probe_activation_cost &&
        obstacle_probe_index < reference.size())
      {
        lateral_obstacle_costs = sampleLateralCostsLocked(
          reference[obstacle_probe_index].pose.pose, costmap,
          dead_zone_lateral_probe_distance_);
      }
      const bool side_bias_obstacle_is_ahead =
        highest_obstacle_cost > weights_.side_preference_activation_cost;
      updateSidePreference(side_bias_obstacle_is_ahead, lateral_obstacle_costs);
    }
  }
  const Eigen::Vector2d measured_velocity(
    velocity.linear.x, velocity.angular.z);
  MPCSidePreference mpc_side_preference = MPCSidePreference::NONE;
  if (control_state_ == ControlState::MPC_TRACKING) {
    if (avoidance_direction_ == AvoidanceDirection::LEFT) {
      mpc_side_preference = MPCSidePreference::LEFT;
    } else if (avoidance_direction_ == AvoidanceDirection::RIGHT) {
      mpc_side_preference = MPCSidePreference::RIGHT;
    }
  }
  // Gửi sai số, quỹ đạo và vận tốc hiện tại vào bộ giải MPC.
  auto result = solver_.solve(
    error, reference, measured_velocity, time_step_, weights_, active_limits,
    MPCSolverSettings{}, obstacle_samples, mpc_side_preference);
  if (!result.solved) {
    // OSQP lỗi hoặc không tìm được nghiệm thì trả lệnh dừng an toàn.
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 2000,
      "MPC solver failed (%s, iterations=%d, solve=%.3f ms); commanding zero velocity",
      result.status.c_str(), result.iterations, result.solve_time_ms);
    recordSolverMetrics(result, compute_start);
    return command;
  }
  std_msgs::msg::Header debug_header = robot_pose.header;
  debug_header.stamp = node->now();
  auto predicted_poses = buildPredictedPoses(reference, result);
  if (costmap_ros_) {
    const auto costmap_frame = costmap_ros_->getGlobalFrameID();
    if (robot_pose.header.frame_id != costmap_frame) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node->get_clock(), 2000,
        "Cannot collision-check MPC trajectory: pose frame '%s' differs from costmap frame '%s'",
        robot_pose.header.frame_id.c_str(), costmap_frame.c_str());
      recordSolverMetrics(result, compute_start);
      return command;
    }
    bool collision_free = isPredictedTrajectoryCollisionFree(predicted_poses);
    if (!collision_free) {
      auto * costmap = costmap_ros_->getCostmap();
      std::vector<MPCObstacleSample> relinearized_samples;
      if (costmap != nullptr) {
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
        relinearized_samples = sampleObstacleCostsLocked(
          predicted_poses, costmap, active_limits.obstacle_cost_limit);
      }
      if (relinearized_samples.size() == result.predicted_errors.size()) {
        for (std::size_t k = 0; k < relinearized_samples.size(); ++k) {
          relinearized_samples[k].linearization_error =
            result.predicted_errors[k].head<2>();
        }
        auto relinearized_result = solver_.solve(
          error, reference, measured_velocity, time_step_, weights_, active_limits,
          MPCSolverSettings{}, relinearized_samples, mpc_side_preference);
        if (relinearized_result.solved) {
          relinearized_result.solve_time_ms += result.solve_time_ms;
          result = relinearized_result;
          predicted_poses = buildPredictedPoses(reference, result);
          collision_free = isPredictedTrajectoryCollisionFree(predicted_poses);
          RCLCPP_INFO_THROTTLE(
            logger_, *node->get_clock(), 2000,
            "MPC relinearized obstacle constraints; second trajectory collision_free=%s",
            collision_free ? "true" : "false");
        } else {
          RCLCPP_WARN_THROTTLE(
            logger_, *node->get_clock(), 2000,
            "MPC relinearized solve failed (%s, iterations=%d)",
            relinearized_result.status.c_str(), relinearized_result.iterations);
        }
      }
    }
    if (!collision_free) {
      publishDebugPaths(reference, predicted_poses, debug_header);
      recordSolverMetrics(result, compute_start);
      updateCollisionState(true);
      const bool obstacle_is_ahead =
        highest_obstacle_cost > active_limits.obstacle_cost_limit;
      if (control_state_ == ControlState::MPC_TRACKING && obstacle_is_ahead &&
        obstacle_probe_index < reference.size())
      {
        lockAvoidanceDirection(lateral_obstacle_costs);
        const double path_world_yaw =
          tf2::getYaw(reference[obstacle_probe_index].pose.pose.orientation);
        startAvoidanceRotation(robot_pose, path_world_yaw);
      }
      RCLCPP_WARN_THROTTLE(
        logger_, *node->get_clock(), 1000,
        "MPC predicted footprint collision; commanding zero velocity");
      return command;
    }
    updateCollisionState(false);
  }
  publishDebugPaths(reference, predicted_poses, debug_header);
  // Lấy điều khiển đầu tiên và chặn lại theo giới hạn cứng.
  const Eigen::Vector2d limited_control = clampCommand(
    result.control, measured_velocity, active_limits);
  command.twist.linear.x = limited_control[0];
  command.twist.angular.z = limited_control[1];
  const bool obstacle_is_ahead =
    highest_obstacle_cost > active_limits.obstacle_cost_limit;
  const bool linear_motion_is_low =
    control_state_ == ControlState::MPC_TRACKING &&
    std::abs(command.twist.linear.x) <= dead_zone_linear_speed_threshold_ &&
    std::abs(velocity.linear.x) <= dead_zone_linear_speed_threshold_;
  const bool goal_is_far = goal_distance > xy_tolerance + goal_position_hysteresis_;
  if (updateDeadZoneState(linear_motion_is_low, goal_is_far, obstacle_is_ahead) &&
    obstacle_probe_index < reference.size())
  {
    lockAvoidanceDirection(lateral_obstacle_costs);
    const double path_world_yaw =
      tf2::getYaw(reference[obstacle_probe_index].pose.pose.orientation);
    startAvoidanceRotation(robot_pose, path_world_yaw);
    const char * avoidance_side =
      avoidance_direction_ == AvoidanceDirection::LEFT ? "left" : "right";
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 2000,
      "MPC dead zone: cycles=%d front_cost=%.3f left_cost=%.3f right_cost=%.3f "
      "locked_side=%s",
      consecutive_dead_zone_cycles_, highest_obstacle_cost,
      lateral_obstacle_costs.x(), lateral_obstacle_costs.y(), avoidance_side);
  }
  recordSolverMetrics(result, compute_start);
  const double compute_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - compute_start).count();
  RCLCPP_INFO_THROTTLE(
    logger_, *node->get_clock(), 2000,
    "MPC status=%s iter=%d solve=%.3f ms total=%.3f ms "
    "error=[%.3f %.3f %.3f] command=[%.3f %.3f]",
    result.status.c_str(), result.iterations, result.solve_time_ms, compute_time_ms,
    error[0], error[1], error[2], command.twist.linear.x, command.twist.angular.z);
  return command;
}

void MPCController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(limits_mutex_);
  // Nav2 định nghĩa 0.0 là NO_SPEED_LIMIT cho cả hai mode.
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    limits_ = configured_limits_;
  } else if (!std::isfinite(speed_limit) || speed_limit < 0.0) {
    RCLCPP_WARN(
      logger_, "Ignoring invalid speed limit %.3f for %s",
      speed_limit, plugin_name_.c_str());
    return;
  } else if (percentage) {
    limits_.max_linear_velocity = configured_limits_.max_linear_velocity *
      std::clamp(speed_limit, 0.0, 100.0) / 100.0;
  } else {
    limits_.max_linear_velocity = std::clamp(
      speed_limit, 0.0,
      configured_limits_.max_linear_velocity);
  }
  if (speed_limit != nav2_costmap_2d::NO_SPEED_LIMIT) {
    limits_.min_linear_velocity = std::min(
      configured_limits_.min_linear_velocity,
      limits_.max_linear_velocity);
  }
}

}  // namespace bumperbot_motion

PLUGINLIB_EXPORT_CLASS(bumperbot_motion::MPCController, nav2_core::Controller)
