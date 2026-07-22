#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "bumperbot_motion/mpc_solver.hpp"

namespace
{

constexpr double kDt = 0.05;
constexpr std::size_t kHorizon = 10U;

std::vector<bumperbot_motion::ReferencePoint> straightReference(
  double linear_velocity = 0.15)
{
  std::vector<bumperbot_motion::ReferencePoint> reference(kHorizon + 1U);
  for (auto & point : reference) {
    point.linear_velocity = linear_velocity;
  }
  return reference;
}

Eigen::Vector3d predictedByDynamics(
  const Eigen::Vector3d & error,
  const Eigen::Vector2d & delta_control,
  const bumperbot_motion::ReferencePoint & reference)
{
  Eigen::Matrix3d dynamics = Eigen::Matrix3d::Identity();
  dynamics(0, 1) = kDt * reference.angular_velocity;
  dynamics(1, 0) = -kDt * reference.angular_velocity;
  dynamics(1, 2) = kDt * reference.linear_velocity;
  Eigen::Matrix<double, 3, 2> input = Eigen::Matrix<double, 3, 2>::Zero();
  input(0, 0) = -kDt;
  input(2, 1) = -kDt;
  return dynamics * error + input * delta_control;
}

}  // namespace

TEST(MPCSolver, zero_error_tracks_reference_with_near_zero_delta_control)
{
  bumperbot_motion::MPCSolver solver;
  const auto reference = straightReference();
  const auto result = solver.solve(
    Eigen::Vector3d::Zero(), reference, Eigen::Vector2d(0.15, 0.0),
    kDt, bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{});

  ASSERT_TRUE(result.solved) << result.status;
  ASSERT_EQ(result.delta_controls.size(), kHorizon);
  EXPECT_NEAR(result.delta_controls.front()[0], 0.0, 1e-4);
  EXPECT_NEAR(result.delta_controls.front()[1], 0.0, 1e-4);
}

TEST(MPCSolver, longitudinal_error_commands_expected_direction)
{
  bumperbot_motion::MPCSolver solver;
  const auto result = solver.solve(
    Eigen::Vector3d(0.10, 0.0, 0.0), straightReference(),
    Eigen::Vector2d(0.15, 0.0), kDt,
    bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{});

  ASSERT_TRUE(result.solved) << result.status;
  EXPECT_GT(result.control[0], 0.15);
}

TEST(MPCSolver, lateral_and_yaw_errors_command_expected_turn_direction)
{
  bumperbot_motion::MPCSolver solver;
  const auto lateral_result = solver.solve(
    Eigen::Vector3d(0.0, 0.10, 0.0), straightReference(),
    Eigen::Vector2d(0.15, 0.0), kDt,
    bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{});
  const auto yaw_result = solver.solve(
    Eigen::Vector3d(0.0, 0.0, 0.10), straightReference(),
    Eigen::Vector2d(0.15, 0.0), kDt,
    bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{});

  ASSERT_TRUE(lateral_result.solved) << lateral_result.status;
  ASSERT_TRUE(yaw_result.solved) << yaw_result.status;
  EXPECT_GT(lateral_result.control[1], 0.0);
  EXPECT_GT(yaw_result.control[1], 0.0);
}

TEST(MPCSolver, equality_dynamics_residual_is_small)
{
  bumperbot_motion::MPCSolver solver;
  auto reference = straightReference();
  for (auto & point : reference) {
    point.angular_velocity = 0.10;
  }
  const auto result = solver.solve(
    Eigen::Vector3d(0.08, -0.04, 0.06), reference,
    Eigen::Vector2d(0.15, 0.10), kDt,
    bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{});

  ASSERT_TRUE(result.solved) << result.status;
  ASSERT_EQ(result.predicted_errors.size(), kHorizon + 1U);
  for (std::size_t k = 0; k < kHorizon; ++k) {
    const Eigen::Vector3d expected = predictedByDynamics(
      result.predicted_errors[k], result.delta_controls[k], reference[k]);
    EXPECT_LT((result.predicted_errors[k + 1U] - expected).lpNorm<Eigen::Infinity>(), 1e-4);
  }
}

TEST(MPCSolver, velocity_and_acceleration_bounds_hold_for_entire_solution)
{
  bumperbot_motion::MPCSolver solver;
  bumperbot_motion::MPCLimits limits;
  const Eigen::Vector2d current_velocity(0.10, -0.20);
  const auto result = solver.solve(
    Eigen::Vector3d(0.20, 0.10, 0.20), straightReference(), current_velocity,
    kDt, bumperbot_motion::MPCWeights{}, limits);

  ASSERT_TRUE(result.solved) << result.status;
  Eigen::Vector2d previous = current_velocity;
  for (const auto & control : result.control_sequence) {
    EXPECT_GE(control[0], limits.min_linear_velocity - 1e-4);
    EXPECT_LE(control[0], limits.max_linear_velocity + 1e-4);
    EXPECT_LE(std::abs(control[1]), limits.max_angular_velocity + 1e-4);
    EXPECT_LE(std::abs(control[0] - previous[0]), limits.max_linear_acceleration * kDt + 2e-4);
    EXPECT_LE(
      std::abs(control[1] - previous[1]), limits.max_angular_acceleration * kDt + 2e-4);
    previous = control;
  }
}

TEST(MPCSolver, changed_reference_uses_measured_velocity_for_first_rate_bound)
{
  bumperbot_motion::MPCSolver solver;
  bumperbot_motion::MPCLimits limits;
  const Eigen::Vector2d measured_velocity(0.19, 0.30);
  auto changed_reference = straightReference(0.05);
  for (auto & point : changed_reference) {
    point.angular_velocity = -0.10;
  }
  const auto result = solver.solve(
    Eigen::Vector3d(0.10, 0.05, 0.10), changed_reference, measured_velocity,
    kDt, bumperbot_motion::MPCWeights{}, limits);

  ASSERT_TRUE(result.solved) << result.status;
  EXPECT_LE(
    std::abs(result.control[0] - measured_velocity[0]),
    limits.max_linear_acceleration * kDt + 2e-4);
  EXPECT_LE(
    std::abs(result.control[1] - measured_velocity[1]),
    limits.max_angular_acceleration * kDt + 2e-4);
}

TEST(MPCSolver, terminal_prediction_reduces_tracking_error)
{
  bumperbot_motion::MPCSolver solver;
  const auto result = solver.solve(
    Eigen::Vector3d(0.15, 0.0, 0.0), straightReference(),
    Eigen::Vector2d(0.15, 0.0), kDt,
    bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{});

  ASSERT_TRUE(result.solved) << result.status;
  ASSERT_FALSE(result.predicted_errors.empty());
  EXPECT_LT(result.predicted_errors.back().norm(), result.predicted_errors.front().norm());
}

TEST(MPCSolver, infeasible_problem_fails_safely)
{
  bumperbot_motion::MPCSolver solver;
  const auto result = solver.solve(
    Eigen::Vector3d::Zero(), straightReference(), Eigen::Vector2d(-1.0, 0.0),
    kDt, bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{});

  EXPECT_FALSE(result.solved);
  EXPECT_NE(result.status.find("infeasible"), std::string::npos);
  EXPECT_TRUE(result.control.allFinite());
}

TEST(MPCSolver, maximum_iteration_status_fails_safely)
{
  bumperbot_motion::MPCSolver solver;
  bumperbot_motion::MPCSolverSettings settings;
  settings.max_iterations = 1;
  settings.absolute_tolerance = 1e-12;
  settings.relative_tolerance = 1e-12;
  const auto result = solver.solve(
    Eigen::Vector3d(0.20, 0.20, 0.50), straightReference(),
    Eigen::Vector2d::Zero(), kDt,
    bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{}, settings);

  EXPECT_FALSE(result.solved);
  EXPECT_NE(result.status.find("maximum iterations"), std::string::npos);
  EXPECT_TRUE(result.control.allFinite());
}

TEST(MPCSolver, nan_and_invalid_inputs_fail_safely)
{
  bumperbot_motion::MPCSolver solver;
  Eigen::Vector3d invalid_error = Eigen::Vector3d::Zero();
  invalid_error[1] = std::numeric_limits<double>::quiet_NaN();
  auto reference = straightReference();
  reference[2].angular_velocity = std::numeric_limits<double>::infinity();

  const auto nan_result = solver.solve(
    invalid_error, straightReference(), Eigen::Vector2d::Zero(), kDt,
    bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{});
  const auto reference_result = solver.solve(
    Eigen::Vector3d::Zero(), reference, Eigen::Vector2d::Zero(), kDt,
    bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{});
  const auto empty_result = solver.solve(
    Eigen::Vector3d::Zero(), {}, Eigen::Vector2d::Zero(), kDt,
    bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{});

  EXPECT_FALSE(nan_result.solved);
  EXPECT_FALSE(reference_result.solved);
  EXPECT_FALSE(empty_result.solved);
  EXPECT_TRUE(nan_result.control.allFinite());
  EXPECT_TRUE(reference_result.control.allFinite());
  EXPECT_TRUE(empty_result.control.allFinite());
}

TEST(MPCSolver, invalid_limits_weights_and_settings_fail_safely)
{
  bumperbot_motion::MPCSolver solver;
  bumperbot_motion::MPCLimits invalid_limits;
  invalid_limits.max_linear_acceleration = 0.0;
  bumperbot_motion::MPCWeights invalid_weights;
  invalid_weights.control[0] = -1.0;
  bumperbot_motion::MPCSolverSettings invalid_settings;
  invalid_settings.absolute_tolerance = 0.0;

  const auto limits_result = solver.solve(
    Eigen::Vector3d::Zero(), straightReference(), Eigen::Vector2d::Zero(), kDt,
    bumperbot_motion::MPCWeights{}, invalid_limits);
  const auto weights_result = solver.solve(
    Eigen::Vector3d::Zero(), straightReference(), Eigen::Vector2d::Zero(), kDt,
    invalid_weights, bumperbot_motion::MPCLimits{});
  const auto settings_result = solver.solve(
    Eigen::Vector3d::Zero(), straightReference(), Eigen::Vector2d::Zero(), kDt,
    bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{}, invalid_settings);

  EXPECT_FALSE(limits_result.solved);
  EXPECT_FALSE(weights_result.solved);
  EXPECT_FALSE(settings_result.solved);
  EXPECT_EQ(limits_result.status, "invalid MPC input");
  EXPECT_EQ(weights_result.status, "invalid MPC input");
  EXPECT_EQ(settings_result.status, "invalid MPC input");
}
