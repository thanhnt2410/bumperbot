#include "bumperbot_motion/mpc_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
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
    weights_.control.minCoeff() < 0.0 || weights_.control_rate.minCoeff() < 0.0)
  {
    invalid("all MPC weights must be finite and non-negative");
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
}

void MPCController::activate()
{
  // Plugin bắt đầu nhận yêu cầu tính vận tốc từ Nav2.
  has_last_pose_stamp_ = false;
  reference_path_pub_->on_activate();
  predicted_path_pub_->on_activate();
  RCLCPP_INFO(logger_, "Activating %s", plugin_name_.c_str());
}

void MPCController::deactivate()
{
  // Plugin tạm dừng khi Controller Server chuyển lifecycle state.
  reference_path_pub_->on_deactivate();
  predicted_path_pub_->on_deactivate();
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
  tf_.reset();
  costmap_ros_.reset();
  has_last_pose_stamp_ = false;
}

void MPCController::setPlan(const nav_msgs::msg::Path & path)
{
  // Lưu đường đi mới do planner của Nav2 gửi tới.
  const bool start_new_goal = global_plan_.poses.empty() || control_state_ == ControlState::STOPPED;
  global_plan_ = path;
  reference_generator_.setPlan(path);
  if (start_new_goal) {
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
  double yaw_error, const MPCLimits & limits) const
{
  auto command = zeroCommand(robot_pose);
  const Eigen::Vector2d desired(
    0.0, goal_angular_gain_ * yaw_error);
  const Eigen::Vector2d current(velocity.linear.x, velocity.angular.z);
  const Eigen::Vector2d limited = clampCommand(desired, current, limits);
  command.twist.linear.x = limited[0];
  command.twist.angular.z = limited[1];
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

void MPCController::publishDebugPaths(
  const std::vector<ReferencePoint> & reference,
  const MPCResult & result,
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
  predicted_path.poses.reserve(result.predicted_errors.size());
  for (std::size_t k = 0; k < result.predicted_errors.size() && k < reference.size(); ++k) {
    const auto & ref_pose = reference[k].pose.pose;
    const auto & error = result.predicted_errors[k];
    const double predicted_yaw = normalizeAngle(tf2::getYaw(ref_pose.orientation) - error[2]);
    geometry_msgs::msg::PoseStamped predicted_pose;
    predicted_pose.header = header;
    predicted_pose.pose.position.x = ref_pose.position.x -
      std::cos(predicted_yaw) * error[0] + std::sin(predicted_yaw) * error[1];
    predicted_pose.pose.position.y = ref_pose.position.y -
      std::sin(predicted_yaw) * error[0] - std::cos(predicted_yaw) * error[1];
    predicted_pose.pose.orientation.z = std::sin(0.5 * predicted_yaw);
    predicted_pose.pose.orientation.w = std::cos(0.5 * predicted_yaw);
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

  // Tạo N + 1 điểm tham chiếu phía trước robot.
  const auto reference = reference_generator_.generate(
    transformed, robot_pose, horizon_, time_step_, reference_velocity_,
    velocity.linear.x, active_limits.max_linear_acceleration,
    active_limits.max_linear_velocity, active_limits.max_angular_velocity);
  if (reference.size() != static_cast<std::size_t>(horizon_ + 1)) {
    RCLCPP_WARN(logger_, "MPC could not generate a reference trajectory");
    return command;
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
  // Gửi sai số, quỹ đạo và vận tốc hiện tại vào bộ giải MPC.
  const auto result = solver_.solve(
    error, reference, Eigen::Vector2d(
      velocity.linear.x,
      velocity.angular.z), time_step_, weights_, active_limits);
  if (!result.solved) {
    // OSQP lỗi hoặc không tìm được nghiệm thì trả lệnh dừng an toàn.
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 2000,
      "MPC solver failed (%s, iterations=%d, solve=%.3f ms); commanding zero velocity",
      result.status.c_str(), result.iterations, result.solve_time_ms);
    return command;
  }
  std_msgs::msg::Header debug_header = robot_pose.header;
  debug_header.stamp = node->now();
  publishDebugPaths(reference, result, debug_header);
  // Lấy điều khiển đầu tiên và chặn lại theo giới hạn cứng.
  const Eigen::Vector2d limited_control = clampCommand(
    result.control, Eigen::Vector2d(velocity.linear.x, velocity.angular.z), active_limits);
  command.twist.linear.x = limited_control[0];
  command.twist.angular.z = limited_control[1];
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
