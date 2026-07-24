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

std::vector<bumperbot_motion::MPCObstacleSample> obstacleSamples(
  double cost, const Eigen::Vector2d & world_gradient)
{
  std::vector<bumperbot_motion::MPCObstacleSample> samples(kHorizon + 1U);
  for (auto & sample : samples) {
    sample.cost = cost;
    sample.world_gradient = world_gradient;
  }
  return samples;
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

TEST(MPCSolver, reuses_workspace_until_reset)
{
  bumperbot_motion::MPCSolver solver;
  const auto first = solver.solve(
    Eigen::Vector3d::Zero(), straightReference(), Eigen::Vector2d(0.15, 0.0),
    kDt, bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{});
  auto curved_reference = straightReference();
  for (auto & point : curved_reference) {
    point.angular_velocity = 0.1;
  }
  const auto second = solver.solve(
    Eigen::Vector3d::Zero(), curved_reference, Eigen::Vector2d(0.15, 0.1),
    kDt, bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{});
  solver.reset();
  const auto after_reset = solver.solve(
    Eigen::Vector3d::Zero(), straightReference(), Eigen::Vector2d(0.15, 0.0),
    kDt, bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{});

  ASSERT_TRUE(first.solved && second.solved && after_reset.solved);
  EXPECT_FALSE(first.workspace_reused);
  EXPECT_TRUE(second.workspace_reused);
  EXPECT_FALSE(after_reset.workspace_reused);
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

TEST(MPCSolver, obstacle_gradient_pushes_prediction_away_from_higher_cost)
{
  bumperbot_motion::MPCSolver solver;
  const auto obstacle_samples = obstacleSamples(0.4, Eigen::Vector2d(0.0, 1.0));
  const auto result = solver.solve(
    Eigen::Vector3d::Zero(), straightReference(), Eigen::Vector2d(0.15, 0.0),
    kDt, bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{},
    bumperbot_motion::MPCSolverSettings{}, obstacle_samples);

  ASSERT_TRUE(result.solved) << result.status;
  ASSERT_EQ(result.obstacle_error_gradients.size(), kHorizon + 1U);
  ASSERT_EQ(result.predicted_errors.size(), kHorizon + 1U);
  EXPECT_LT(result.control[1], 0.0);
  EXPECT_GT(result.predicted_errors.back().y(), 0.0);
}

TEST(MPCSolver, high_flat_obstacle_cost_uses_slack_and_remains_feasible)
{
  bumperbot_motion::MPCSolver solver;
  const auto obstacle_samples = obstacleSamples(0.8, Eigen::Vector2d::Zero());
  const auto result = solver.solve(
    Eigen::Vector3d::Zero(), straightReference(), Eigen::Vector2d(0.15, 0.0),
    kDt, bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{},
    bumperbot_motion::MPCSolverSettings{}, obstacle_samples);

  ASSERT_TRUE(result.solved) << result.status;
  ASSERT_EQ(result.obstacle_slacks.size(), kHorizon);
  for (const double slack : result.obstacle_slacks) {
    EXPECT_NEAR(slack, 0.3, 1e-3);
  }
}

TEST(MPCSolver, useful_obstacle_gradient_reduces_total_slack)
{
  bumperbot_motion::MPCWeights weights;
  weights.obstacle_cost = 0.0;
  weights.obstacle_slack_weight = 100.0;
  const auto flat_samples = obstacleSamples(0.8, Eigen::Vector2d::Zero());
  const auto directed_samples = obstacleSamples(0.8, Eigen::Vector2d(0.0, 10.0));

  bumperbot_motion::MPCSolver flat_solver;
  const auto flat_result = flat_solver.solve(
    Eigen::Vector3d::Zero(), straightReference(), Eigen::Vector2d(0.15, 0.0),
    kDt, weights, bumperbot_motion::MPCLimits{},
    bumperbot_motion::MPCSolverSettings{}, flat_samples);
  bumperbot_motion::MPCSolver directed_solver;
  const auto directed_result = directed_solver.solve(
    Eigen::Vector3d::Zero(), straightReference(), Eigen::Vector2d(0.15, 0.0),
    kDt, weights, bumperbot_motion::MPCLimits{},
    bumperbot_motion::MPCSolverSettings{}, directed_samples);

  ASSERT_TRUE(flat_result.solved) << flat_result.status;
  ASSERT_TRUE(directed_result.solved) << directed_result.status;
  double flat_slack_sum = 0.0;
  double directed_slack_sum = 0.0;
  for (const double slack : flat_result.obstacle_slacks) {
    flat_slack_sum += slack;
  }
  for (const double slack : directed_result.obstacle_slacks) {
    directed_slack_sum += slack;
  }
  EXPECT_LT(directed_result.control[1], 0.0);
  EXPECT_LT(directed_slack_sum, flat_slack_sum - 1e-3);
}

TEST(MPCSolver, relinearized_obstacle_sample_preserves_same_affine_cost)
{
  const Eigen::Vector2d world_gradient(0.0, 1.0);
  const auto reference_samples = obstacleSamples(0.8, world_gradient);
  auto predicted_samples = obstacleSamples(0.7, world_gradient);
  for (auto & sample : predicted_samples) {
    sample.linearization_error.y() = 0.1;
  }

  bumperbot_motion::MPCSolver reference_solver;
  const auto reference_result = reference_solver.solve(
    Eigen::Vector3d::Zero(), straightReference(), Eigen::Vector2d(0.15, 0.0),
    kDt, bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{},
    bumperbot_motion::MPCSolverSettings{}, reference_samples);
  bumperbot_motion::MPCSolver predicted_solver;
  const auto predicted_result = predicted_solver.solve(
    Eigen::Vector3d::Zero(), straightReference(), Eigen::Vector2d(0.15, 0.0),
    kDt, bumperbot_motion::MPCWeights{}, bumperbot_motion::MPCLimits{},
    bumperbot_motion::MPCSolverSettings{}, predicted_samples);

  ASSERT_TRUE(reference_result.solved) << reference_result.status;
  ASSERT_TRUE(predicted_result.solved) << predicted_result.status;
  EXPECT_TRUE(reference_result.control.isApprox(predicted_result.control, 1e-4));
  ASSERT_EQ(reference_result.obstacle_slacks.size(), predicted_result.obstacle_slacks.size());
  for (std::size_t k = 0; k < reference_result.obstacle_slacks.size(); ++k) {
    EXPECT_NEAR(
      reference_result.obstacle_slacks[k],
      predicted_result.obstacle_slacks[k], 1e-4);
  }
}

TEST(MPCSolver, left_preference_makes_predicted_lateral_error_negative)
{
  bumperbot_motion::MPCWeights weights;
  weights.side_preference_cost = 0.2;
  weights.side_preference_activation_cost = 0.5;
  const auto obstacle_samples = obstacleSamples(0.8, Eigen::Vector2d::Zero());
  bumperbot_motion::MPCSolver solver;
  const auto result = solver.solve(
    Eigen::Vector3d::Zero(), straightReference(), Eigen::Vector2d(0.15, 0.0),
    kDt, weights, bumperbot_motion::MPCLimits{},
    bumperbot_motion::MPCSolverSettings{}, obstacle_samples,
    bumperbot_motion::MPCSidePreference::LEFT);

  ASSERT_TRUE(result.solved) << result.status;
  EXPECT_LT(result.predicted_errors.back().y(), -1e-6);
}

TEST(MPCSolver, right_preference_makes_predicted_lateral_error_positive)
{
  bumperbot_motion::MPCWeights weights;
  weights.side_preference_cost = 0.2;
  weights.side_preference_activation_cost = 0.5;
  const auto obstacle_samples = obstacleSamples(0.8, Eigen::Vector2d::Zero());
  bumperbot_motion::MPCSolver solver;
  const auto result = solver.solve(
    Eigen::Vector3d::Zero(), straightReference(), Eigen::Vector2d(0.15, 0.0),
    kDt, weights, bumperbot_motion::MPCLimits{},
    bumperbot_motion::MPCSolverSettings{}, obstacle_samples,
    bumperbot_motion::MPCSidePreference::RIGHT);

  ASSERT_TRUE(result.solved) << result.status;
  EXPECT_GT(result.predicted_errors.back().y(), 1e-6);
}

TEST(MPCSolver, side_preference_is_inactive_below_obstacle_threshold)
{
  bumperbot_motion::MPCWeights weights;
  weights.side_preference_cost = 0.2;
  weights.side_preference_activation_cost = 0.5;
  const auto low_cost_samples = obstacleSamples(0.4, Eigen::Vector2d::Zero());
  bumperbot_motion::MPCSolver baseline_solver;
  const auto baseline = baseline_solver.solve(
    Eigen::Vector3d::Zero(), straightReference(), Eigen::Vector2d(0.15, 0.0),
    kDt, weights, bumperbot_motion::MPCLimits{},
    bumperbot_motion::MPCSolverSettings{}, low_cost_samples);
  bumperbot_motion::MPCSolver preferred_solver;
  const auto preferred = preferred_solver.solve(
    Eigen::Vector3d::Zero(), straightReference(), Eigen::Vector2d(0.15, 0.0),
    kDt, weights, bumperbot_motion::MPCLimits{},
    bumperbot_motion::MPCSolverSettings{}, low_cost_samples,
    bumperbot_motion::MPCSidePreference::LEFT);

  ASSERT_TRUE(baseline.solved && preferred.solved);
  EXPECT_TRUE(baseline.control.isApprox(preferred.control, 1e-6));
  EXPECT_TRUE(
    baseline.predicted_errors.back().isApprox(
      preferred.predicted_errors.back(), 1e-6));
}
