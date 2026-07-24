#include <atomic>
#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "bumperbot_motion/mpc_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace bumperbot_motion
{

class MPCControllerTestPeer
{
public:
  static bool poseIsFree(
    MPCController & controller, const geometry_msgs::msg::Pose & pose,
    nav2_costmap_2d::Costmap2D * costmap, const nav2_costmap_2d::Footprint & footprint)
  {
    return controller.isFootprintCollisionFreeLocked(pose, costmap, footprint);
  }

  static bool trajectoryIsFree(
    MPCController & controller, const std::vector<geometry_msgs::msg::Pose> & poses,
    nav2_costmap_2d::Costmap2D * costmap, const nav2_costmap_2d::Footprint & footprint)
  {
    return controller.isPredictedTrajectoryCollisionFreeLocked(poses, costmap, footprint);
  }

  static bool obstacleCostCorridorIsClear(
    MPCController & controller, const std::vector<geometry_msgs::msg::Pose> & poses,
    nav2_costmap_2d::Costmap2D * costmap, const nav2_costmap_2d::Footprint & footprint,
    double maximum_normalized_cost)
  {
    return controller.isObstacleCostCorridorClearLocked(
      poses, costmap, footprint, maximum_normalized_cost);
  }

  static std::vector<MPCObstacleSample> obstacleSamples(
    MPCController & controller, const std::vector<geometry_msgs::msg::Pose> & poses,
    nav2_costmap_2d::Costmap2D * costmap, double cost_limit)
  {
    return controller.sampleObstacleCostsLocked(poses, costmap, cost_limit);
  }

  static bool updateDeadZone(
    MPCController & controller, bool low_motion, bool far_goal, bool obstacle)
  {
    return controller.updateDeadZoneState(low_motion, far_goal, obstacle);
  }

  static int lockAvoidance(MPCController & controller, const Eigen::Vector2d & costs)
  {
    controller.lockAvoidanceDirection(costs);
    return static_cast<int>(controller.avoidance_direction_);
  }

  static int updateSidePreference(
    MPCController & controller, bool obstacle_is_ahead, const Eigen::Vector2d & costs)
  {
    controller.updateSidePreference(obstacle_is_ahead, costs);
    return static_cast<int>(controller.avoidance_direction_);
  }

  static double startAvoidance(
    MPCController & controller, const geometry_msgs::msg::PoseStamped & robot_pose,
    double path_world_yaw = 0.0)
  {
    controller.startAvoidanceRotation(robot_pose, path_world_yaw);
    return controller.avoidance_target_world_yaw_;
  }

  static void buildAvoidanceReference(
    MPCController & controller, const geometry_msgs::msg::PoseStamped & robot_pose,
    std::vector<ReferencePoint> & reference)
  {
    controller.buildAvoidanceReference(robot_pose, reference);
  }

  static bool avoidanceTravelComplete(
    MPCController & controller, const geometry_msgs::msg::PoseStamped & robot_pose)
  {
    return controller.avoidanceTravelComplete(robot_pose);
  }

  static void setRejoinTravel(
    MPCController & controller, const Eigen::Vector2d & start_world_position,
    double rejoin_world_yaw, double rejoin_distance)
  {
    controller.avoidance_phase_ = MPCController::AvoidancePhase::REJOIN_PATH;
    controller.avoidance_start_world_position_ = start_world_position;
    controller.avoidance_target_world_yaw_ = rejoin_world_yaw;
    controller.avoidance_rejoin_distance_ = rejoin_distance;
  }

  static std::vector<geometry_msgs::msg::Pose> rejoinPoses(
    MPCController & controller, const geometry_msgs::msg::PoseStamped & robot_pose)
  {
    return controller.buildAvoidanceRejoinPoses(robot_pose);
  }

  static std::vector<geometry_msgs::msg::Pose> forwardPoses(
    MPCController & controller, const geometry_msgs::msg::PoseStamped & robot_pose)
  {
    return controller.buildAvoidanceForwardPoses(robot_pose);
  }

  static void setPolicy(MPCController & controller, bool unknown, bool outside)
  {
    controller.consider_unknown_as_obstacle_ = unknown;
    controller.consider_outside_costmap_as_obstacle_ = outside;
  }

  static void setFailurePolicy(MPCController & controller, int cycles, double timeout)
  {
    controller.collision_failure_cycles_ = cycles;
    controller.collision_failure_timeout_ = timeout;
    controller.resetCollisionState();
  }

  static void update(MPCController & controller, bool collision)
  {
    controller.updateCollisionState(collision);
  }

  static int cycles(const MPCController & controller)
  {
    return controller.consecutive_collision_cycles_;
  }

  static void ageTimer(MPCController & controller, double seconds)
  {
    controller.collision_start_time_ =
      std::chrono::steady_clock::now() - std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(std::chrono::duration<double>(seconds));
  }

};

}  // namespace bumperbot_motion

namespace
{

nav2_costmap_2d::Footprint squareFootprint()
{
  nav2_costmap_2d::Footprint footprint(4);
  footprint[0].x = 0.1; footprint[0].y = 0.1;
  footprint[1].x = 0.1; footprint[1].y = -0.1;
  footprint[2].x = -0.1; footprint[2].y = -0.1;
  footprint[3].x = -0.1; footprint[3].y = 0.1;
  return footprint;
}

void setWorldCost(
  nav2_costmap_2d::Costmap2D & costmap, double x, double y, unsigned char cost)
{
  unsigned int mx = 0, my = 0;
  ASSERT_TRUE(costmap.worldToMap(x, y, mx, my));
  costmap.setCost(mx, my, cost);
}

}  // namespace

TEST(MPCCollision, applies_lethal_unknown_and_outside_policy)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, -2.5, -2.5);
  bumperbot_motion::MPCController controller;
  const auto footprint = squareFootprint();
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap.getMutex());

  EXPECT_TRUE(
    bumperbot_motion::MPCControllerTestPeer::poseIsFree(
      controller, pose, &costmap, footprint));
  setWorldCost(costmap, 0.1, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(
    bumperbot_motion::MPCControllerTestPeer::poseIsFree(
      controller, pose, &costmap, footprint));
  setWorldCost(costmap, 0.1, 0.0, nav2_costmap_2d::NO_INFORMATION);
  bumperbot_motion::MPCControllerTestPeer::setPolicy(controller, false, true);
  EXPECT_TRUE(
    bumperbot_motion::MPCControllerTestPeer::poseIsFree(
      controller, pose, &costmap, footprint));
  pose.position.x = 3.0;
  EXPECT_FALSE(
    bumperbot_motion::MPCControllerTestPeer::poseIsFree(
      controller, pose, &costmap, footprint));
}

TEST(MPCCollision, persistent_collision_uses_cycles_timeout_and_safe_reset)
{
  bumperbot_motion::MPCController controller;
  using Peer = bumperbot_motion::MPCControllerTestPeer;
  Peer::setFailurePolicy(controller, 3, 10.0);
  EXPECT_NO_THROW(Peer::update(controller, true));
  Peer::update(controller, false);
  EXPECT_EQ(Peer::cycles(controller), 0);
  EXPECT_NO_THROW(Peer::update(controller, true));
  EXPECT_NO_THROW(Peer::update(controller, true));
  EXPECT_THROW(Peer::update(controller, true), nav2_core::PlannerException);

  Peer::setFailurePolicy(controller, 100, 0.1);
  Peer::update(controller, true);
  Peer::ageTimer(controller, 1.0);
  EXPECT_THROW(Peer::update(controller, true), nav2_core::PlannerException);
}

TEST(MPCCollision, interpolation_detects_obstacle_between_predicted_poses)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, -2.5, -2.5);
  bumperbot_motion::MPCController controller;
  const auto footprint = squareFootprint();
  std::vector<geometry_msgs::msg::Pose> poses(2);
  poses[0].position.x = -0.4;
  poses[1].position.x = 0.4;
  poses[0].orientation.w = 1.0;
  poses[1].orientation.w = 1.0;
  setWorldCost(costmap, 0.0, 0.1, nav2_costmap_2d::LETHAL_OBSTACLE);
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap.getMutex());

  using Peer = bumperbot_motion::MPCControllerTestPeer;
  EXPECT_TRUE(Peer::poseIsFree(controller, poses.front(), &costmap, footprint));
  EXPECT_TRUE(Peer::poseIsFree(controller, poses.back(), &costmap, footprint));
  EXPECT_FALSE(Peer::trajectoryIsFree(controller, poses, &costmap, footprint));
}

TEST(MPCCollision, repeated_checks_share_costmap_mutex_without_deadlock)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, -2.5, -2.5);
  bumperbot_motion::MPCController controller;
  const auto footprint = squareFootprint();
  std::vector<geometry_msgs::msg::Pose> poses(2);
  poses[0].orientation.w = 1.0;
  poses[1].position.x = 0.3;
  poses[1].orientation.w = 1.0;
  unsigned int update_x = 0, update_y = 0;
  ASSERT_TRUE(costmap.worldToMap(1.5, 1.5, update_x, update_y));
  std::atomic_bool checks_free{true};

  std::thread writer([&]() {
      for (int iteration = 0; iteration < 1000; ++iteration) {
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap.getMutex());
        costmap.setCost(
          update_x, update_y, iteration % 2 == 0 ?
          nav2_costmap_2d::LETHAL_OBSTACLE : nav2_costmap_2d::FREE_SPACE);
      }
    });
  std::thread checker([&]() {
      for (int iteration = 0; iteration < 1000; ++iteration) {
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap.getMutex());
        if (!bumperbot_motion::MPCControllerTestPeer::trajectoryIsFree(
          controller, poses, &costmap, footprint))
        {
          checks_free = false;
        }
      }
    });
  writer.join();
  checker.join();
  EXPECT_TRUE(checks_free);
}

TEST(MPCCollision, lateral_probe_recovers_gradient_from_locally_flat_cost)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, -2.5, -2.5);
  bumperbot_motion::MPCController controller;
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;

  setWorldCost(costmap, 0.0, 0.0, 200);
  setWorldCost(costmap, 0.05, 0.0, 200);
  setWorldCost(costmap, -0.05, 0.0, 200);
  setWorldCost(costmap, 0.0, 0.05, 200);
  setWorldCost(costmap, 0.0, -0.05, 200);
  setWorldCost(costmap, 0.0, 0.15, nav2_costmap_2d::FREE_SPACE);
  setWorldCost(costmap, 0.0, -0.15, nav2_costmap_2d::LETHAL_OBSTACLE);

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap.getMutex());
  const auto samples = bumperbot_motion::MPCControllerTestPeer::obstacleSamples(
    controller, {pose}, &costmap, 0.5);

  ASSERT_EQ(samples.size(), 1U);
  EXPECT_GT(samples.front().cost, 0.5);
  EXPECT_NEAR(samples.front().world_gradient.x(), 0.0, 1e-9);
  EXPECT_LT(samples.front().world_gradient.y(), 0.0);
}

TEST(MPCCollision, dead_zone_requires_consecutive_low_motion_far_goal_and_obstacle)
{
  bumperbot_motion::MPCController controller;
  using Peer = bumperbot_motion::MPCControllerTestPeer;

  for (int cycle = 1; cycle < 10; ++cycle) {
    EXPECT_FALSE(Peer::updateDeadZone(controller, true, true, true));
  }
  EXPECT_TRUE(Peer::updateDeadZone(controller, true, true, true));
  EXPECT_FALSE(Peer::updateDeadZone(controller, false, true, true));
  EXPECT_FALSE(Peer::updateDeadZone(controller, true, false, true));
  EXPECT_FALSE(Peer::updateDeadZone(controller, true, true, false));
}

TEST(MPCCollision, avoidance_direction_selects_lower_cost_and_stays_locked)
{
  bumperbot_motion::MPCController controller;
  using Peer = bumperbot_motion::MPCControllerTestPeer;

  EXPECT_EQ(Peer::lockAvoidance(controller, Eigen::Vector2d(0.8, 0.2)), -1);
  EXPECT_EQ(Peer::lockAvoidance(controller, Eigen::Vector2d(0.1, 0.9)), -1);
}

TEST(MPCCollision, side_preference_unlocks_after_consecutive_clear_cycles)
{
  bumperbot_motion::MPCController controller;
  using Peer = bumperbot_motion::MPCControllerTestPeer;

  EXPECT_EQ(Peer::updateSidePreference(controller, true, Eigen::Vector2d(0.2, 0.8)), 1);
  for (int cycle = 1; cycle < 10; ++cycle) {
    EXPECT_EQ(Peer::updateSidePreference(controller, false, Eigen::Vector2d::Zero()), 1);
  }
  EXPECT_EQ(Peer::updateSidePreference(controller, false, Eigen::Vector2d::Zero()), 0);
}

TEST(MPCCollision, avoidance_rotation_target_uses_locked_side)
{
  bumperbot_motion::MPCController controller;
  using Peer = bumperbot_motion::MPCControllerTestPeer;
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.pose.orientation.w = 1.0;

  Peer::lockAvoidance(controller, Eigen::Vector2d(0.8, 0.2));
  EXPECT_NEAR(Peer::startAvoidance(controller, robot_pose), -M_PI_2, 1e-9);
}

TEST(MPCCollision, avoidance_reference_follows_locked_heading_at_reduced_speed)
{
  bumperbot_motion::MPCController controller;
  using Peer = bumperbot_motion::MPCControllerTestPeer;
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.pose.orientation.w = 1.0;
  std::vector<bumperbot_motion::ReferencePoint> reference(2);

  Peer::lockAvoidance(controller, Eigen::Vector2d(0.8, 0.2));
  Peer::startAvoidance(controller, robot_pose);
  Peer::buildAvoidanceReference(controller, robot_pose, reference);

  EXPECT_GT(reference.back().pose.pose.position.x, 0.0);
  EXPECT_LT(reference.back().pose.pose.position.y, 0.0);
  EXPECT_NEAR(reference.front().linear_velocity, 0.15, 1e-9);
  EXPECT_NEAR(tf2::getYaw(reference.back().pose.pose.orientation), -M_PI_2, 1e-9);
}

TEST(MPCCollision, avoidance_progress_uses_motion_along_phase_heading)
{
  bumperbot_motion::MPCController controller;
  using Peer = bumperbot_motion::MPCControllerTestPeer;
  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.pose.orientation.w = 1.0;
  Peer::lockAvoidance(controller, Eigen::Vector2d(0.2, 0.8));
  const double avoidance_yaw = Peer::startAvoidance(controller, start_pose);

  geometry_msgs::msg::PoseStamped current_pose = start_pose;
  current_pose.pose.position.x = -0.8 * std::sin(avoidance_yaw);
  current_pose.pose.position.y = 0.8 * std::cos(avoidance_yaw);
  EXPECT_FALSE(Peer::avoidanceTravelComplete(controller, current_pose));

  current_pose.pose.position.x = 0.71 * std::cos(avoidance_yaw);
  current_pose.pose.position.y = 0.71 * std::sin(avoidance_yaw);
  EXPECT_TRUE(Peer::avoidanceTravelComplete(controller, current_pose));
}

TEST(MPCCollision, rejoin_progress_uses_the_measured_rejoin_distance)
{
  bumperbot_motion::MPCController controller;
  using Peer = bumperbot_motion::MPCControllerTestPeer;
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.pose.orientation.w = 1.0;
  Peer::setRejoinTravel(controller, Eigen::Vector2d::Zero(), M_PI_2, 0.8);

  robot_pose.pose.position.y = 0.79;
  EXPECT_FALSE(Peer::avoidanceTravelComplete(controller, robot_pose));
  robot_pose.pose.position.y = 0.81;
  EXPECT_TRUE(Peer::avoidanceTravelComplete(controller, robot_pose));
}

TEST(MPCCollision, rejoin_corridor_exposes_obstacle_between_offset_and_global_path)
{
  bumperbot_motion::MPCController controller;
  using Peer = bumperbot_motion::MPCControllerTestPeer;
  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.pose.orientation.w = 1.0;
  Peer::lockAvoidance(controller, Eigen::Vector2d(0.2, 0.8));
  Peer::startAvoidance(controller, start_pose, 0.0);

  geometry_msgs::msg::PoseStamped current_pose = start_pose;
  current_pose.pose.position.x = 1.2;
  current_pose.pose.position.y = 0.7;
  const auto rejoin_poses = Peer::rejoinPoses(controller, current_pose);
  ASSERT_EQ(rejoin_poses.size(), 11U);
  EXPECT_NEAR(rejoin_poses.back().position.x, 1.2, 1e-9);
  EXPECT_NEAR(rejoin_poses.back().position.y, 0.0, 1e-9);

  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, -2.5, -2.5);
  const auto footprint = squareFootprint();
  setWorldCost(costmap, 1.2, 0.35, nav2_costmap_2d::LETHAL_OBSTACLE);
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap.getMutex());
  EXPECT_FALSE(Peer::trajectoryIsFree(controller, rejoin_poses, &costmap, footprint));
}

TEST(MPCCollision, forward_corridor_detects_obstacle_after_nominal_lateral_clearance)
{
  bumperbot_motion::MPCController controller;
  using Peer = bumperbot_motion::MPCControllerTestPeer;
  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.pose.orientation.w = 1.0;
  Peer::lockAvoidance(controller, Eigen::Vector2d(0.2, 0.8));
  Peer::startAvoidance(controller, start_pose, 0.0);

  geometry_msgs::msg::PoseStamped offset_pose = start_pose;
  offset_pose.pose.position.y = 0.7;
  const auto forward_poses = Peer::forwardPoses(controller, offset_pose);
  ASSERT_EQ(forward_poses.size(), 21U);
  EXPECT_NEAR(forward_poses.back().position.x, 1.1, 1e-9);
  EXPECT_NEAR(forward_poses.back().position.y, 0.7, 1e-9);

  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, -2.5, -2.5);
  const auto footprint = squareFootprint();
  setWorldCost(costmap, 0.5, 0.7, nav2_costmap_2d::LETHAL_OBSTACLE);
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap.getMutex());
  EXPECT_FALSE(Peer::trajectoryIsFree(controller, forward_poses, &costmap, footprint));
}

TEST(MPCCollision, corridor_remains_blocked_by_cost_below_collision_threshold)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, -2.5, -2.5);
  bumperbot_motion::MPCController controller;
  const auto footprint = squareFootprint();
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  setWorldCost(costmap, 0.0, 0.0, 200);
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap.getMutex());

  using Peer = bumperbot_motion::MPCControllerTestPeer;
  EXPECT_TRUE(Peer::trajectoryIsFree(controller, {pose}, &costmap, footprint));
  EXPECT_FALSE(
    Peer::obstacleCostCorridorIsClear(
      controller, {pose}, &costmap, footprint, 0.5));
}
