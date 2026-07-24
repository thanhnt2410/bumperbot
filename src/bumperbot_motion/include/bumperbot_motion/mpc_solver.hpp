#ifndef BUMPERBOT_MOTION__MPC_SOLVER_HPP_
#define BUMPERBOT_MOTION__MPC_SOLVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "bumperbot_motion/reference_trajectory.hpp"

namespace bumperbot_motion
{

enum class MPCSidePreference
{
  RIGHT = -1,
  NONE = 0,
  LEFT = 1
};

struct MPCWeights
{
  Eigen::Vector3d state{10.0, 20.0, 5.0};
  Eigen::Vector3d terminal{20.0, 40.0, 10.0};
  Eigen::Vector2d control{0.5, 0.2};
  Eigen::Vector2d control_rate{2.0, 0.5};
  double obstacle_cost{2.0};
  double obstacle_slack_weight{100.0};
  double side_preference_cost{0.0};
  double side_preference_activation_cost{0.5};
};

struct MPCObstacleSample
{
  double cost{0.0};
  // Gradient trong hệ tọa độ world: chiều tăng nhanh nhất của costmap cost.
  Eigen::Vector2d world_gradient{Eigen::Vector2d::Zero()};
  // Sai số [dọc, ngang] nơi cost và gradient được lấy.
  // Lần solve đầu lấy mẫu trên reference nên giá trị mặc định bằng zero.
  Eigen::Vector2d linearization_error{Eigen::Vector2d::Zero()};
  double linearization_yaw{0.0};
};

struct MPCLimits
{
  double min_linear_velocity{0.0};
  double max_linear_velocity{0.2};
  double max_angular_velocity{0.8};
  double max_linear_acceleration{0.5};
  double max_angular_acceleration{2.0};
  double obstacle_cost_limit{0.5};
};

struct MPCResult
{
  bool solved{false};
  bool workspace_reused{false};
  Eigen::Vector2d control{Eigen::Vector2d::Zero()};
  std::string status;
  int status_value{0};
  int iterations{0};
  double solve_time_ms{0.0};
  std::vector<Eigen::Vector3d> predicted_errors;
  std::vector<Eigen::Vector2d> delta_controls;
  std::vector<Eigen::Vector2d> control_sequence;
  std::vector<Eigen::Vector2d> obstacle_error_gradients; //lưu gradient sau khi tính toán costmap cost, dùng để tính toán costmap cost cho các điểm sample của obstacle
  std::vector<double> obstacle_slacks;
};

struct MPCSolverSettings
{
  int max_iterations{2000};
  double absolute_tolerance{1e-4};
  double relative_tolerance{1e-4};
};

class MPCSolver
{
public:
  MPCSolver();
  ~MPCSolver();
  MPCSolver(const MPCSolver &) = delete;
  MPCSolver & operator=(const MPCSolver &) = delete;

  void reset();
  MPCResult solve(
    const Eigen::Vector3d & initial_error,
    const std::vector<ReferencePoint> & reference,
    const Eigen::Vector2d & current_velocity,
    double time_step, const MPCWeights & weights, const MPCLimits & limits,
    const MPCSolverSettings & settings = MPCSolverSettings{},
    const std::vector<MPCObstacleSample> & obstacle_samples = {},
    MPCSidePreference side_preference = MPCSidePreference::NONE);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bumperbot_motion

#endif  // BUMPERBOT_MOTION__MPC_SOLVER_HPP_
