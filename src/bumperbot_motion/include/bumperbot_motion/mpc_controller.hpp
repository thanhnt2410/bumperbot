#ifndef BUMPERBOT_MOTION__MPC_CONTROLLER_HPP_
#define BUMPERBOT_MOTION__MPC_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"

#include "bumperbot_motion/mpc_solver.hpp"
#include "bumperbot_motion/reference_trajectory.hpp"

namespace bumperbot_motion
{

class MPCController : public nav2_core::Controller
{
public:
  MPCController() = default;
  ~MPCController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  enum class ControlState
  {
    ROTATE_TO_PATH,
    MPC_TRACKING,
    GOAL_ALIGN,
    STOPPED
  };

  bool transformPlan(const std::string & target_frame, nav_msgs::msg::Path & transformed) const;
  geometry_msgs::msg::TwistStamped zeroCommand(
    const geometry_msgs::msg::PoseStamped & robot_pose) const;
  geometry_msgs::msg::TwistStamped rotationCommand(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & velocity,
    double yaw_error, const MPCLimits & limits) const;
  Eigen::Vector2d clampCommand(
    const Eigen::Vector2d & desired, const Eigen::Vector2d & current,
    const MPCLimits & limits) const;
  void validateParameters() const;
  bool runtimeInputValid(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & velocity) const;
  void publishDebugPaths(
    const std::vector<ReferencePoint> & reference,
    const MPCResult & result,
    const std_msgs::msg::Header & header) const;
  static double normalizeAngle(double angle);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPCController")};
  std::string plugin_name_;
  nav_msgs::msg::Path global_plan_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;
  ReferenceTrajectory reference_generator_;
  MPCSolver solver_;
  MPCWeights weights_;
  MPCLimits limits_;
  MPCLimits configured_limits_;
  mutable std::mutex limits_mutex_;
  int horizon_{10};
  double time_step_{0.05};
  double reference_velocity_{0.15};
  double goal_distance_tolerance_{0.25};
  double goal_yaw_tolerance_{0.25};
  double rotate_to_path_enter_angle_{0.785};
  double rotate_to_path_exit_angle_{0.35};
  double goal_angular_gain_{1.5};
  double goal_position_hysteresis_{0.05};
  mutable rclcpp::Time last_pose_stamp_{0, 0, RCL_ROS_TIME};
  mutable bool has_last_pose_stamp_{false};
  ControlState control_state_{ControlState::ROTATE_TO_PATH};
};

}  // namespace bumperbot_motion

#endif  // BUMPERBOT_MOTION__MPC_CONTROLLER_HPP_
