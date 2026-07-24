#ifndef BUMPERBOT_MOTION__MPC_CONTROLLER_HPP_
#define BUMPERBOT_MOTION__MPC_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"

#include "bumperbot_motion/mpc_solver.hpp"
#include "bumperbot_motion/reference_trajectory.hpp"

namespace bumperbot_motion
{

class MPCController : public nav2_core::Controller
{
  friend class MPCControllerTestPeer;

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
    AVOID_ROTATE,
    AVOID_TRACKING,
    GOAL_ALIGN,
    STOPPED
  };

  enum class AvoidanceDirection
  {
    RIGHT = -1,
    NONE = 0,
    LEFT = 1
  };

  enum class AvoidancePhase
  {
    CLEAR_LATERAL,
    PASS_OBSTACLE,
    REJOIN_PATH
  };

  bool transformPlan(const std::string & target_frame, nav_msgs::msg::Path & transformed) const;
  geometry_msgs::msg::TwistStamped zeroCommand(
    const geometry_msgs::msg::PoseStamped & robot_pose) const;
  geometry_msgs::msg::TwistStamped rotationCommand(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & velocity,
    double yaw_error, const MPCLimits & limits);
  Eigen::Vector2d clampCommand(
    const Eigen::Vector2d & desired, const Eigen::Vector2d & current,
    const MPCLimits & limits) const;
  void validateParameters() const;
  bool runtimeInputValid(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & velocity) const;
  std::vector<geometry_msgs::msg::Pose> buildPredictedPoses(
    const std::vector<ReferencePoint> & reference, const MPCResult & result) const;
  std::vector<geometry_msgs::msg::Pose> simulateCommandPoses(
    const geometry_msgs::msg::Pose & start_pose,
    const geometry_msgs::msg::TwistStamped & command) const;
  bool isFootprintCollisionFreeLocked(
    const geometry_msgs::msg::Pose & pose, nav2_costmap_2d::Costmap2D * costmap,
    const nav2_costmap_2d::Footprint & footprint) const;
  bool isPredictedTrajectoryCollisionFreeLocked(
    const std::vector<geometry_msgs::msg::Pose> & poses,
    nav2_costmap_2d::Costmap2D * costmap,
    const nav2_costmap_2d::Footprint & footprint) const;
  bool isObstacleCostCorridorClearLocked(
    const std::vector<geometry_msgs::msg::Pose> & poses,
    nav2_costmap_2d::Costmap2D * costmap,
    const nav2_costmap_2d::Footprint & footprint,
    double maximum_normalized_cost) const;
  std::vector<MPCObstacleSample> sampleObstacleCostsLocked(
    const std::vector<geometry_msgs::msg::Pose> & poses,
    nav2_costmap_2d::Costmap2D * costmap, double obstacle_cost_limit) const;
  double normalizedCostAtWorldLocked(
    double world_x, double world_y, nav2_costmap_2d::Costmap2D * costmap) const;
  Eigen::Vector2d calculateFallbackObstacleGradientLocked(
    const geometry_msgs::msg::Pose & pose,
    nav2_costmap_2d::Costmap2D * costmap, double probe_distance) const;
  Eigen::Vector2d sampleLateralCostsLocked(
    const geometry_msgs::msg::Pose & pose,
    nav2_costmap_2d::Costmap2D * costmap, double probe_distance) const;
  bool updateDeadZoneState(
    bool linear_motion_is_low, bool goal_is_far, bool obstacle_is_ahead);
  void lockAvoidanceDirection(const Eigen::Vector2d & lateral_costs);
  void updateSidePreference(
    bool obstacle_is_ahead, const Eigen::Vector2d & lateral_costs);
  void startAvoidanceRotation(
    const geometry_msgs::msg::PoseStamped & robot_pose, double path_world_yaw);
  void buildAvoidanceReference(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    std::vector<ReferencePoint> & reference) const;
  bool avoidanceTravelComplete(
    const geometry_msgs::msg::PoseStamped & robot_pose) const;
  std::vector<geometry_msgs::msg::Pose> buildAvoidanceRejoinPoses(
    const geometry_msgs::msg::PoseStamped & robot_pose) const;
  std::vector<geometry_msgs::msg::Pose> buildAvoidanceForwardPoses(
    const geometry_msgs::msg::PoseStamped & robot_pose) const;
  bool isPredictedTrajectoryCollisionFree(
    const std::vector<geometry_msgs::msg::Pose> & poses) const;
  bool isObstacleCostCorridorClear(
    const std::vector<geometry_msgs::msg::Pose> & poses,
    double maximum_normalized_cost) const;
  void resetCollisionState();
  void updateCollisionState(bool collision);
  void publishCollisionState(bool collision) const;
  void publishDebugPaths(
    const std::vector<ReferencePoint> & reference,
    const std::vector<geometry_msgs::msg::Pose> & predicted_poses,
    const std_msgs::msg::Header & header) const;
  void recordSolverMetrics(
    const MPCResult & result,
    const std::chrono::steady_clock::time_point & compute_start);
  void resetSolverMetrics();
  static double normalizeAngle(double angle);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPCController")};
  std::string plugin_name_;
  nav_msgs::msg::Path global_plan_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr collision_pub_;
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
  int collision_cost_threshold_{253};
  bool consider_unknown_as_obstacle_{true};
  bool consider_outside_costmap_as_obstacle_{true};
  int collision_failure_cycles_{10};
  double collision_failure_timeout_{1.0};
  int consecutive_collision_cycles_{0};
  bool collision_timer_active_{false};
  std::chrono::steady_clock::time_point collision_start_time_;
  double dead_zone_linear_speed_threshold_{0.01};
  double dead_zone_lateral_probe_distance_{0.35};
  int dead_zone_detection_cycles_{10};
  int consecutive_dead_zone_cycles_{0};
  int side_preference_clear_cycles_{10};
  int consecutive_side_preference_clear_cycles_{0};
  AvoidanceDirection avoidance_direction_{AvoidanceDirection::NONE};
  AvoidancePhase avoidance_phase_{AvoidancePhase::CLEAR_LATERAL};
  double avoidance_rotation_angle_{M_PI_2};
  double avoidance_yaw_tolerance_{0.10};
  double avoidance_target_world_yaw_{0.0};
  double avoidance_path_world_yaw_{0.0};
  Eigen::Vector2d avoidance_path_world_position_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d avoidance_start_world_position_{Eigen::Vector2d::Zero()};
  double avoidance_lateral_clearance_{0.70};
  double avoidance_pass_distance_{1.10};
  double avoidance_rejoin_distance_{0.0};
  double avoidance_reference_velocity_{0.15};
  mutable rclcpp::Time last_pose_stamp_{0, 0, RCL_ROS_TIME};
  mutable bool has_last_pose_stamp_{false};
  std::vector<double> solve_time_samples_;
  std::vector<double> callback_time_samples_;
  std::vector<int> iteration_samples_;
  std::size_t solver_sample_count_{0};
  std::size_t solver_failure_count_{0};
  ControlState control_state_{ControlState::ROTATE_TO_PATH};
};

}  // namespace bumperbot_motion

#endif  // BUMPERBOT_MOTION__MPC_CONTROLLER_HPP_
