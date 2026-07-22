#ifndef BUMPERBOT_MOTION__REFERENCE_TRAJECTORY_HPP_
#define BUMPERBOT_MOTION__REFERENCE_TRAJECTORY_HPP_

#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace bumperbot_motion
{

struct ReferencePoint
{
  geometry_msgs::msg::PoseStamped pose;
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  double arc_length{0.0};
  double curvature{0.0};
};

class ReferenceTrajectory
{
public:
  void setPlan(const nav_msgs::msg::Path & plan);
  void reset();

  std::vector<ReferencePoint> generate(
    const nav_msgs::msg::Path & plan,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    int horizon, double time_step, double reference_velocity);

  std::vector<ReferencePoint> generate(
    const nav_msgs::msg::Path & plan,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    int horizon, double time_step, double reference_velocity,
    double current_linear_velocity, double max_linear_acceleration,
    double max_angular_velocity);

private:
  void preparePlan(const nav_msgs::msg::Path & plan);
  double interpolate(const std::vector<double> & values, double arc_length) const;
  std::size_t segmentForArcLength(double arc_length) const;
  static double normalizeAngle(double angle);

  nav_msgs::msg::Path source_plan_;
  std::vector<double> arc_lengths_;
  std::vector<double> tracking_yaws_;
  std::vector<double> curvatures_;
  std::size_t progress_segment_{0U};
  bool has_progress_{false};
};

}  // namespace bumperbot_motion

#endif  // BUMPERBOT_MOTION__REFERENCE_TRAJECTORY_HPP_
