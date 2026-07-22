#ifndef BUMPERBOT_MOTION__MPC_SOLVER_HPP_
#define BUMPERBOT_MOTION__MPC_SOLVER_HPP_

#include <string>
#include <vector>

#include <Eigen/Core>

#include "bumperbot_motion/reference_trajectory.hpp"

namespace bumperbot_motion
{

struct MPCWeights
{
  Eigen::Vector3d state{10.0, 20.0, 5.0};
  Eigen::Vector3d terminal{20.0, 40.0, 10.0};
  Eigen::Vector2d control{0.5, 0.2};
  Eigen::Vector2d control_rate{2.0, 0.5};
};

struct MPCLimits
{
  double min_linear_velocity{0.0};
  double max_linear_velocity{0.2};
  double max_angular_velocity{0.8};
  double max_linear_acceleration{0.5};
  double max_angular_acceleration{2.0};
};

struct MPCResult
{
  bool solved{false};
  Eigen::Vector2d control{Eigen::Vector2d::Zero()};
  std::string status;
  int status_value{0};
  int iterations{0};
  double solve_time_ms{0.0};
  std::vector<Eigen::Vector3d> predicted_errors;
  std::vector<Eigen::Vector2d> delta_controls;
  std::vector<Eigen::Vector2d> control_sequence;
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
  MPCResult solve(
    const Eigen::Vector3d & initial_error,
    const std::vector<ReferencePoint> & reference,
    const Eigen::Vector2d & current_velocity,
    double time_step, const MPCWeights & weights, const MPCLimits & limits,
    const MPCSolverSettings & settings = MPCSolverSettings{}) const;
};

}  // namespace bumperbot_motion

#endif  // BUMPERBOT_MOTION__MPC_SOLVER_HPP_
