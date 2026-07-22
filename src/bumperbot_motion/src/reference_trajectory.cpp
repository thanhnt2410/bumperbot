#include "bumperbot_motion/reference_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bumperbot_motion
{

double ReferenceTrajectory::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

void ReferenceTrajectory::reset()
{
  source_plan_.poses.clear();
  arc_lengths_.clear();
  tracking_yaws_.clear();
  curvatures_.clear();
  progress_segment_ = 0U;
  has_progress_ = false;
}

void ReferenceTrajectory::setPlan(const nav_msgs::msg::Path & plan)
{
  preparePlan(plan);
  progress_segment_ = 0U;
  has_progress_ = false;
}

void ReferenceTrajectory::preparePlan(const nav_msgs::msg::Path & plan)
{
  source_plan_ = plan;
  const std::size_t count = plan.poses.size();
  arc_lengths_.assign(count, 0.0);
  tracking_yaws_.assign(count, 0.0);
  curvatures_.assign(count, 0.0);
  if (count == 0U) {
    return;
  }

  for (std::size_t i = 1U; i < count; ++i) {
    const auto & previous = plan.poses[i - 1U].pose.position;
    const auto & current = plan.poses[i].pose.position;
    arc_lengths_[i] = arc_lengths_[i - 1U] +
      std::hypot(current.x - previous.x, current.y - previous.y);
  }

  // Khi tracking dùng hướng segment. Orientation cuối path chỉ dành cho goal yaw.
  for (std::size_t i = 0U; i + 1U < count; ++i) {
    std::size_t next = i + 1U;
    while (next + 1U < count &&
      arc_lengths_[next] - arc_lengths_[i] < 1e-9)
    {
      ++next;
    }
    const auto & first = plan.poses[i].pose.position;
    const auto & second = plan.poses[next].pose.position;
    if (std::hypot(second.x - first.x, second.y - first.y) > 1e-9) {
      tracking_yaws_[i] = std::atan2(second.y - first.y, second.x - first.x);
    } else if (i > 0U) {
      tracking_yaws_[i] = tracking_yaws_[i - 1U];
    } else {
      tracking_yaws_[i] = tf2::getYaw(plan.poses[i].pose.orientation);
    }
    if (i > 0U) {
      tracking_yaws_[i] = tracking_yaws_[i - 1U] +
        normalizeAngle(tracking_yaws_[i] - tracking_yaws_[i - 1U]);
    }
  }
  if (count == 1U) {
    tracking_yaws_.front() = tf2::getYaw(plan.poses.front().pose.orientation);
  } else {
    // Không đưa goal orientation vào tracking curvature. Goal yaw được dùng
    // riêng tại endpoint bởi state GOAL_ALIGN của controller.
    tracking_yaws_.back() = tracking_yaws_[count - 2U];
  }

  std::vector<double> raw_curvature(count, 0.0);
  for (std::size_t i = 1U; i + 1U < count; ++i) {
    const double distance = arc_lengths_[i + 1U] - arc_lengths_[i - 1U];
    if (distance > 1e-9) {
      raw_curvature[i] =
        (tracking_yaws_[i + 1U] - tracking_yaws_[i - 1U]) / distance;
    }
  }
  if (count > 1U) {
    raw_curvature.front() = raw_curvature[1U];
    raw_curvature.back() = raw_curvature[count - 2U];
  }
  // Bộ lọc moving average nhỏ để curvature không spike tại vertex.
  for (std::size_t i = 0U; i < count; ++i) {
    const std::size_t first = i > 0U ? i - 1U : i;
    const std::size_t last = std::min(i + 1U, count - 1U);
    double sum = 0.0;
    for (std::size_t j = first; j <= last; ++j) {
      sum += raw_curvature[j];
    }
    curvatures_[i] = sum / static_cast<double>(last - first + 1U);
  }
}

std::size_t ReferenceTrajectory::segmentForArcLength(double arc_length) const
{
  auto upper = std::lower_bound(arc_lengths_.begin(), arc_lengths_.end(), arc_length);
  std::size_t second = static_cast<std::size_t>(std::distance(arc_lengths_.begin(), upper));
  return std::clamp<std::size_t>(second, 1U, arc_lengths_.size() - 1U) - 1U;
}

double ReferenceTrajectory::interpolate(
  const std::vector<double> & values, double arc_length) const
{
  const std::size_t first = segmentForArcLength(arc_length);
  const std::size_t second = first + 1U;
  const double length = arc_lengths_[second] - arc_lengths_[first];
  const double ratio = length > 1e-9 ?
    std::clamp((arc_length - arc_lengths_[first]) / length, 0.0, 1.0) : 0.0;
  return values[first] + ratio * (values[second] - values[first]);
}

std::vector<ReferencePoint> ReferenceTrajectory::generate(
  const nav_msgs::msg::Path & plan, const geometry_msgs::msg::PoseStamped & robot_pose,
  int horizon, double time_step, double reference_velocity)
{
  return generate(
    plan, robot_pose, horizon, time_step, reference_velocity,
    reference_velocity, std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity());
}

std::vector<ReferencePoint> ReferenceTrajectory::generate(
  const nav_msgs::msg::Path & plan, const geometry_msgs::msg::PoseStamped & robot_pose,
  int horizon, double time_step, double reference_velocity,
  double current_linear_velocity, double max_linear_acceleration,
  double max_angular_velocity)
{
  std::vector<ReferencePoint> result;
  if (plan.poses.empty() || horizon < 1 || time_step <= 0.0) {
    return result;
  }
  if (source_plan_.poses.size() != plan.poses.size() || arc_lengths_.size() != plan.poses.size()) {
    setPlan(plan);
  }
  if (plan.poses.size() == 1U) {
    result.resize(static_cast<std::size_t>(horizon + 1));
    for (auto & point : result) {
      point.pose = plan.poses.front();
    }
    return result;
  }

  // Chỉ cho phép tìm lùi một cửa sổ nhỏ so với progress của chu kỳ trước.
  constexpr std::size_t backward_segment_window = 3U;
  const std::size_t search_start = has_progress_ && progress_segment_ > backward_segment_window ?
    progress_segment_ - backward_segment_window : 0U;
  std::size_t closest_segment = search_start;
  double closest_ratio = 0.0;
  double closest_squared_distance = std::numeric_limits<double>::max();
  for (std::size_t i = search_start; i + 1U < plan.poses.size(); ++i) {
    const auto & first = plan.poses[i].pose.position;
    const auto & second = plan.poses[i + 1U].pose.position;
    const double segment_x = second.x - first.x;
    const double segment_y = second.y - first.y;
    const double squared_length = segment_x * segment_x + segment_y * segment_y;
    const double ratio = squared_length > 1e-12 ? std::clamp(
      ((robot_pose.pose.position.x - first.x) * segment_x +
      (robot_pose.pose.position.y - first.y) * segment_y) / squared_length,
      0.0, 1.0) : 0.0;
    const double dx = robot_pose.pose.position.x - (first.x + ratio * segment_x);
    const double dy = robot_pose.pose.position.y - (first.y + ratio * segment_y);
    const double squared_distance = dx * dx + dy * dy;
    // Khi bằng nhau, ưu tiên segment phía goal để không nhảy về nhánh cũ tại giao điểm.
    if (squared_distance <= closest_squared_distance) {
      closest_squared_distance = squared_distance;
      closest_segment = i;
      closest_ratio = ratio;
    }
  }
  progress_segment_ = closest_segment;
  has_progress_ = true;
  const double segment_length = arc_lengths_[closest_segment + 1U] -
    arc_lengths_[closest_segment];
  double sample_s = arc_lengths_[closest_segment] + closest_ratio * segment_length;

  const double source_yaw = tf2::getYaw(source_plan_.poses.front().pose.orientation);
  const double transformed_yaw = tf2::getYaw(plan.poses.front().pose.orientation);
  const double yaw_offset = normalizeAngle(transformed_yaw - source_yaw);
  double previous_velocity = std::max(0.0, current_linear_velocity);
  result.reserve(static_cast<std::size_t>(horizon + 1));
  for (int k = 0; k <= horizon; ++k) {
    sample_s = std::clamp(sample_s, 0.0, arc_lengths_.back());
    const std::size_t first = segmentForArcLength(sample_s);
    const std::size_t second = first + 1U;
    const double length = arc_lengths_[second] - arc_lengths_[first];
    const double ratio = length > 1e-9 ?
      std::clamp((sample_s - arc_lengths_[first]) / length, 0.0, 1.0) : 0.0;

    ReferencePoint point;
    point.pose.header = plan.header;
    point.pose.pose.position.x = plan.poses[first].pose.position.x +
      ratio * (plan.poses[second].pose.position.x - plan.poses[first].pose.position.x);
    point.pose.pose.position.y = plan.poses[first].pose.position.y +
      ratio * (plan.poses[second].pose.position.y - plan.poses[first].pose.position.y);
    // Reference tracking luôn dùng hướng tiếp tuyến của path, kể cả endpoint.
    // Orientation của goal được MPCController dùng riêng trong GOAL_ALIGN.
    const double yaw = interpolate(tracking_yaws_, sample_s) + yaw_offset;
    point.pose.pose.orientation.z = std::sin(0.5 * yaw);
    point.pose.pose.orientation.w = std::cos(0.5 * yaw);
    point.arc_length = sample_s;
    point.curvature = interpolate(curvatures_, sample_s);

    const double remaining = std::max(0.0, arc_lengths_.back() - sample_s);
    double target_velocity = std::max(0.0, reference_velocity);
    double curvature_velocity_limit = std::numeric_limits<double>::infinity();
    if (std::isfinite(max_angular_velocity) && std::abs(point.curvature) > 1e-6) {
      curvature_velocity_limit = max_angular_velocity / std::abs(point.curvature);
      target_velocity = std::min(target_velocity, curvature_velocity_limit);
    }
    if (std::isfinite(max_linear_acceleration) && max_linear_acceleration > 0.0) {
      target_velocity = std::min(
        target_velocity, std::sqrt(2.0 * max_linear_acceleration * remaining));
      const double maximum_change = max_linear_acceleration * time_step;
      target_velocity = std::clamp(
        target_velocity,
        std::max(0.0, previous_velocity - maximum_change),
        previous_velocity + maximum_change);
    }
    // Angular velocity là hard reference limit; nếu cần phải ưu tiên giảm v.
    target_velocity = std::min(target_velocity, curvature_velocity_limit);
    if (remaining <= 1e-9) {
      target_velocity = 0.0;
    }
    point.linear_velocity = target_velocity;
    point.angular_velocity = target_velocity * point.curvature;
    result.push_back(point);
    previous_velocity = target_velocity;
    sample_s += target_velocity * time_step;
  }
  return result;
}

}  // namespace bumperbot_motion
