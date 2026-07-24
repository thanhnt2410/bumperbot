#include "bumperbot_motion/mpc_solver.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/SparseCore>

extern "C" {
#include "osqp.h"
}

namespace bumperbot_motion
{
struct MPCSolver::Impl
{
  ~Impl()
  {
    if (workspace != nullptr) {osqp_cleanup(workspace);}
  }

  OSQPWorkspace * workspace{nullptr};
  c_int horizon{0};
  c_int p_nonzeros{0};
  c_int a_nonzeros{0};
  int max_iterations{0};
  double absolute_tolerance{0.0};
  double relative_tolerance{0.0};
};

MPCSolver::MPCSolver()
: impl_(std::make_unique<Impl>())
{
}

MPCSolver::~MPCSolver() = default;

void MPCSolver::reset()
{
  impl_ = std::make_unique<Impl>();
}

// Dựng bài toán QP, gọi OSQP và trả về điều khiển đầu tiên [v, omega].
MPCResult MPCSolver::solve(
  const Eigen::Vector3d & initial_error, const std::vector<ReferencePoint> & reference,
  const Eigen::Vector2d & current_velocity, double dt, const MPCWeights & weights,
  const MPCLimits & limits, const MPCSolverSettings & solver_settings,
  const std::vector<MPCObstacleSample> & obstacle_samples,
  MPCSidePreference side_preference)
{
  MPCResult result;

  // Kiểm tra dữ liệu đầu vào trước khi dựng ma trận.
  const bool valid_limits =
    std::isfinite(limits.min_linear_velocity) &&
    std::isfinite(limits.max_linear_velocity) &&
    std::isfinite(limits.max_angular_velocity) &&
    std::isfinite(limits.max_linear_acceleration) &&
    std::isfinite(limits.max_angular_acceleration) &&
    limits.min_linear_velocity <= limits.max_linear_velocity &&
    limits.max_angular_velocity > 0.0 &&
    limits.max_linear_acceleration > 0.0 &&
    limits.max_angular_acceleration > 0.0 &&
    std::isfinite(limits.obstacle_cost_limit) &&
    limits.obstacle_cost_limit >= 0.0 && limits.obstacle_cost_limit <= 1.0;
  const bool valid_weights =
    weights.state.allFinite() && weights.terminal.allFinite() &&
    weights.control.allFinite() && weights.control_rate.allFinite() &&
    weights.state.minCoeff() >= 0.0 && weights.terminal.minCoeff() >= 0.0 &&
    weights.control.minCoeff() >= 0.0 && weights.control_rate.minCoeff() >= 0.0 &&
    std::isfinite(weights.obstacle_cost) && weights.obstacle_cost >= 0.0 &&
    std::isfinite(weights.obstacle_slack_weight) && weights.obstacle_slack_weight > 0.0 &&
    std::isfinite(weights.side_preference_cost) && weights.side_preference_cost >= 0.0 &&
    std::isfinite(weights.side_preference_activation_cost) &&
    weights.side_preference_activation_cost >= 0.0 &&
    weights.side_preference_activation_cost <= 1.0;
  const bool valid_settings =
    solver_settings.max_iterations > 0 &&
    std::isfinite(solver_settings.absolute_tolerance) &&
    std::isfinite(solver_settings.relative_tolerance) &&
    solver_settings.absolute_tolerance > 0.0 && solver_settings.relative_tolerance > 0.0;
  bool valid_obstacles =
    obstacle_samples.empty() || obstacle_samples.size() == reference.size();
  for (const auto & sample : obstacle_samples) {
    valid_obstacles =
      valid_obstacles && std::isfinite(sample.cost) &&
      sample.cost >= 0.0 && sample.cost <= 1.0 &&
      sample.world_gradient.allFinite() &&
      sample.linearization_error.allFinite() &&
      std::isfinite(sample.linearization_yaw);
  }
  if (reference.size() < 2U || !std::isfinite(dt) || dt <= 0.0 ||
    !initial_error.allFinite() ||
    !current_velocity.allFinite() || solver_settings.max_iterations <= 0 ||
    !valid_limits || !valid_weights || !valid_settings || !valid_obstacles)
  {
    result.status = "invalid MPC input";
    return result;
  }
  for (const auto & point : reference) {
    if (!std::isfinite(point.linear_velocity) || !std::isfinite(point.angular_velocity)) {
      result.status = "invalid MPC reference";
      return result;
    }
  }
  result.obstacle_error_gradients.reserve(obstacle_samples.size());
  for (std::size_t k = 0; k < obstacle_samples.size(); ++k) {
    const double yaw = obstacle_samples[k].linearization_yaw;
    const Eigen::Vector2d & world_gradient =
      obstacle_samples[k].world_gradient;

    Eigen::Matrix2d world_to_robot_rotation;
    world_to_robot_rotation <<
      std::cos(yaw), std::sin(yaw),
      -std::sin(yaw), std::cos(yaw);

    const Eigen::Vector2d robot_gradient =
      world_to_robot_rotation * world_gradient;
    const Eigen::Vector2d obstacle_error_gradient =
      -robot_gradient;

    result.obstacle_error_gradients.emplace_back(
      obstacle_error_gradient);
  }
  // Biến tối ưu gồm e_0...e_N, delta_u_0...delta_u_(N-1) và slack_1...slack_N.
  const c_int horizon = static_cast<c_int>(reference.size() - 1U);
  const c_int state_count = 3 * (horizon + 1);
  const c_int control_count = 2 * horizon;
  const c_int slack_count = horizon;
  const c_int control_start = state_count;
  const c_int slack_start = control_start + control_count;
  const c_int variable_count = slack_start + slack_count;
  const c_int equality_count = 3 * (horizon + 1);
  const c_int control_bound_count = 2 * horizon;
  const c_int rate_count = 2 * horizon;
  const c_int slack_bound_start = equality_count + control_bound_count + rate_count;
  const c_int obstacle_constraint_start = slack_bound_start + slack_count;
  const c_int obstacle_constraint_count = horizon;
  const c_int constraint_count = obstacle_constraint_start + obstacle_constraint_count;
  std::vector<c_float> linear_cost(variable_count, 0.0);
  // Ma trận P chứa trọng số sai số trạng thái, điều khiển và độ thay đổi điều khiển.
  std::vector<Eigen::Triplet<double, c_int>> p_entries;
  for (c_int k = 0; k <= horizon; ++k) {
    // Phạt sai số x, y, yaw tại mỗi bước dự đoán.
    for (c_int i = 0; i < 3; ++i) {
      const double weight = k == horizon ? weights.terminal[i] : weights.state[i];
      c_int state = 3 * k + i;
      p_entries.emplace_back(state, state, 2.0 * weight);
    }
  }
  if (!result.obstacle_error_gradients.empty()) {
    for (c_int prediction_step = 1; prediction_step <= horizon; ++prediction_step) {
      const std::size_t gradient_index = static_cast<std::size_t>(prediction_step);
      const Eigen::Vector2d & obstacle_gradient =
        result.obstacle_error_gradients[gradient_index];
      const c_int state_offset = 3 * prediction_step;
      const c_int longitudinal_error_index = state_offset;
      const c_int lateral_error_index = state_offset + 1;
      linear_cost[longitudinal_error_index] +=
        weights.obstacle_cost * obstacle_gradient.x();
      linear_cost[lateral_error_index] +=
        weights.obstacle_cost * obstacle_gradient.y();
    }
  }
  if (side_preference != MPCSidePreference::NONE && !obstacle_samples.empty()) {
    const double side_sign = static_cast<int>(side_preference);
    for (c_int prediction_step = 1; prediction_step <= horizon; ++prediction_step) {
      const std::size_t sample_index = static_cast<std::size_t>(prediction_step);
      if (obstacle_samples[sample_index].cost <= weights.side_preference_activation_cost) {
        continue;
      }
      const double horizon_progress =
        static_cast<double>(prediction_step) / static_cast<double>(horizon);
      const c_int lateral_error_index = 3 * prediction_step + 1;
      linear_cost[lateral_error_index] +=
        side_sign * weights.side_preference_cost * horizon_progress;
    }
  }
  for (c_int slack_index = 0; slack_index < slack_count; ++slack_index) {
    const c_int slack_variable = slack_start + slack_index;
    p_entries.emplace_back(
      slack_variable, slack_variable, 2.0 * weights.obstacle_slack_weight); //1/2 × zᵀPz
  }
  for (c_int i = 0; i < 2; ++i) {
    for (c_int k = 0; k < horizon; ++k) {
      const c_int control = state_count + 2 * k + i;
      p_entries.emplace_back(control, control, 2.0 * weights.control[i]);
    }

    // Rate cost tại k=0 được tính so với measured velocity thực tế.
    const c_int first_control = state_count + i;
    const double first_offset = i == 0 ?
      reference.front().linear_velocity - current_velocity[i] :
      reference.front().angular_velocity - current_velocity[i];
    p_entries.emplace_back(
      first_control, first_control, 2.0 * weights.control_rate[i]);
    linear_cost[first_control] += 2.0 * weights.control_rate[i] * first_offset;

    // Các rate cost còn lại dùng u_k - u_(k-1), bao gồm thay đổi của u_ref.
    for (c_int k = 1; k < horizon; ++k) {
      const c_int previous_control = state_count + 2 * (k - 1) + i;
      const c_int current_control = state_count + 2 * k + i;
      const auto & previous_ref = reference[static_cast<std::size_t>(k - 1)];
      const auto & current_ref = reference[static_cast<std::size_t>(k)];
      const double previous_value = i == 0 ?
        previous_ref.linear_velocity : previous_ref.angular_velocity;
      const double current_value = i == 0 ?
        current_ref.linear_velocity : current_ref.angular_velocity;
      const double offset = current_value - previous_value;
      p_entries.emplace_back(
        previous_control, previous_control, 2.0 * weights.control_rate[i]);
      p_entries.emplace_back(
        current_control, current_control, 2.0 * weights.control_rate[i]);
      p_entries.emplace_back(
        previous_control, current_control, -2.0 * weights.control_rate[i]);
      linear_cost[previous_control] -= 2.0 * weights.control_rate[i] * offset;
      linear_cost[current_control] += 2.0 * weights.control_rate[i] * offset;
    }
  }

  // Ma trận A chứa sai số đầu, động học, giới hạn vận tốc và gia tốc.
  std::vector<Eigen::Triplet<double, c_int>> a_entries;
  std::vector<c_float> lower(constraint_count, 0.0);
  std::vector<c_float> upper(constraint_count, 0.0);
  for (c_int i = 0; i < 3; ++i) {
    // Cố định e_0 bằng sai số hiện tại của robot.
    a_entries.emplace_back(i, i, 1.0);
    lower[i] = upper[i] = initial_error[i];
  }
  for (c_int k = 0; k < horizon; ++k) {
    // Tuyến tính hóa mô hình unicycle quanh v_ref và omega_ref.
    const c_int row = 3 + 3 * k;
    const double v_ref = reference[static_cast<std::size_t>(k)].linear_velocity;
    const double w_ref = reference[static_cast<std::size_t>(k)].angular_velocity;
    Eigen::Matrix3d dynamics = Eigen::Matrix3d::Identity();
    dynamics(0, 1) = dt * w_ref;
    dynamics(1, 0) = -dt * w_ref;
    dynamics(1, 2) = dt * v_ref;
    Eigen::Matrix<double, 3, 2> input = Eigen::Matrix<double, 3, 2>::Zero();
    input(0, 0) = -dt;
    input(2, 1) = -dt;
    for (c_int i = 0; i < 3; ++i) {
      a_entries.emplace_back(row + i, 3 * (k + 1) + i, 1.0);
      for (c_int j = 0; j < 3; ++j) {
        // Giữ cả phần tử zero để sparsity pattern không đổi giữa các chu kỳ.
        a_entries.emplace_back(row + i, 3 * k + j, -dynamics(i, j));
      }
      for (c_int j = 0; j < 2; ++j) {
        if (input(i, j) != 0.0) {
          a_entries.emplace_back(row + i, state_count + 2 * k + j, -input(i, j));
        }
      }
    }
  }
  for (c_int k = 0; k < horizon; ++k) {
    // Thêm giới hạn v, omega và tốc độ thay đổi của chúng.
    const auto & ref = reference[static_cast<std::size_t>(k)];
    const double ref_control[2] = {ref.linear_velocity, ref.angular_velocity};
    const double minimum[2] = {limits.min_linear_velocity, -limits.max_angular_velocity};
    const double maximum[2] = {limits.max_linear_velocity, limits.max_angular_velocity};
    for (c_int i = 0; i < 2; ++i) {
      const c_int bound_row = equality_count + 2 * k + i;
      a_entries.emplace_back(bound_row, state_count + 2 * k + i, 1.0);
      lower[bound_row] = minimum[i] - ref_control[i];
      upper[bound_row] = maximum[i] - ref_control[i];
      const c_int rate_row = equality_count + control_bound_count + 2 * k + i;
      a_entries.emplace_back(rate_row, state_count + 2 * k + i, 1.0);
      const double acceleration = i ==
        0 ? limits.max_linear_acceleration : limits.max_angular_acceleration;
      if (k == 0) {
        // Bước đầu phải so với vận tốc thật đang được Nav2 truyền vào.
        lower[rate_row] = current_velocity[i] - ref_control[i] - acceleration * dt;
        upper[rate_row] = current_velocity[i] - ref_control[i] + acceleration * dt;
      } else {
        // Các bước sau so với điều khiển của bước đứng trước.
        a_entries.emplace_back(rate_row, state_count + 2 * (k - 1) + i, -1.0);
        const auto & previous = reference[static_cast<std::size_t>(k - 1)];
        const double previous_ref = i == 0 ? previous.linear_velocity : previous.angular_velocity;
        lower[rate_row] = previous_ref - ref_control[i] - acceleration * dt;
        upper[rate_row] = previous_ref - ref_control[i] + acceleration * dt;
      }
    }
  }
  for (c_int slack_index = 0; slack_index < slack_count; ++slack_index) {
    const c_int constraint_row = slack_bound_start + slack_index;
    const c_int slack_variable = slack_start + slack_index;
    a_entries.emplace_back(constraint_row, slack_variable, 1.0);
    lower[constraint_row] = 0.0;
    upper[constraint_row] = OSQP_INFTY;
  }
  for (c_int prediction_step = 1; prediction_step <= horizon; ++prediction_step) {
    const c_int obstacle_index = prediction_step - 1;
    const c_int constraint_row = obstacle_constraint_start + obstacle_index;
    const c_int state_offset = 3 * prediction_step;
    const c_int slack_variable = slack_start + obstacle_index;

    const bool has_obstacle_sample = !obstacle_samples.empty();
    const std::size_t sample_index = static_cast<std::size_t>(prediction_step);
    const Eigen::Vector2d obstacle_gradient = has_obstacle_sample ?
      result.obstacle_error_gradients[sample_index] : Eigen::Vector2d::Zero();
    const double sample_cost = has_obstacle_sample ?
      obstacle_samples[sample_index].cost : 0.0;
    const Eigen::Vector2d linearization_error = has_obstacle_sample ?
      obstacle_samples[sample_index].linearization_error :
      Eigen::Vector2d::Zero();
    const double linearized_cost_at_zero =
      sample_cost - obstacle_gradient.dot(linearization_error);

    a_entries.emplace_back(constraint_row, state_offset, obstacle_gradient.x());
    a_entries.emplace_back(constraint_row, state_offset + 1, obstacle_gradient.y());
    a_entries.emplace_back(constraint_row, slack_variable, -1.0);
    lower[constraint_row] = -OSQP_INFTY;
    upper[constraint_row] = has_obstacle_sample ?
      limits.obstacle_cost_limit - linearized_cost_at_zero : OSQP_INFTY;
  }

  // Chuyển danh sách phần tử sang ma trận thưa dạng cột mà OSQP cần.
  Eigen::SparseMatrix<double, Eigen::ColMajor, c_int> p_matrix(
    variable_count, variable_count);
  Eigen::SparseMatrix<double, Eigen::ColMajor, c_int> a_matrix(
    constraint_count, variable_count);
  p_matrix.setFromTriplets(p_entries.begin(), p_entries.end());
  a_matrix.setFromTriplets(a_entries.begin(), a_entries.end());
  p_matrix.makeCompressed();
  a_matrix.makeCompressed();

  // Tạo cấu trúc dữ liệu đầu vào cho OSQP.
  csc * p = csc_matrix(
    variable_count, variable_count, p_matrix.nonZeros(), p_matrix.valuePtr(),
    p_matrix.innerIndexPtr(), p_matrix.outerIndexPtr());
  csc * a = csc_matrix(
    constraint_count, variable_count, a_matrix.nonZeros(), a_matrix.valuePtr(),
    a_matrix.innerIndexPtr(), a_matrix.outerIndexPtr());
  OSQPData data{};
  data.n = variable_count; data.m = constraint_count; data.P = p; data.A = a;
  data.q = linear_cost.data(); data.l = lower.data(); data.u = upper.data();
  OSQPSettings settings;
  // Workspace chỉ được dựng lại khi kích thước hoặc solver settings thay đổi.
  osqp_set_default_settings(&settings);
  settings.verbose = false;
  settings.warm_start = false;
  settings.max_iter = solver_settings.max_iterations;
  settings.eps_abs = solver_settings.absolute_tolerance;
  settings.eps_rel = solver_settings.relative_tolerance;
  const bool rebuild =
    impl_->workspace == nullptr || impl_->horizon != horizon ||
    impl_->p_nonzeros != p_matrix.nonZeros() ||
    impl_->a_nonzeros != a_matrix.nonZeros() ||
    impl_->max_iterations != solver_settings.max_iterations ||
    impl_->absolute_tolerance != solver_settings.absolute_tolerance ||
    impl_->relative_tolerance != solver_settings.relative_tolerance;
  c_int workspace_status = 0;
  if (rebuild) {
    reset();
    workspace_status = osqp_setup(&impl_->workspace, &data, &settings);
    if (workspace_status == 0) {
      impl_->horizon = horizon;
      impl_->p_nonzeros = p_matrix.nonZeros();
      impl_->a_nonzeros = a_matrix.nonZeros();
      impl_->max_iterations = solver_settings.max_iterations;
      impl_->absolute_tolerance = solver_settings.absolute_tolerance;
      impl_->relative_tolerance = solver_settings.relative_tolerance;
    }
  } else {
    workspace_status = osqp_update_P_A(
      impl_->workspace, p_matrix.valuePtr(), OSQP_NULL, p_matrix.nonZeros(),
      a_matrix.valuePtr(), OSQP_NULL, a_matrix.nonZeros());
    if (workspace_status == 0) {
      workspace_status = osqp_update_lin_cost(impl_->workspace, linear_cost.data());
    }
    if (workspace_status == 0) {
      workspace_status = osqp_update_bounds(impl_->workspace, lower.data(), upper.data());
    }
    result.workspace_reused = workspace_status == 0;
  }

  OSQPWorkspace * workspace = impl_->workspace;
  if (workspace_status == 0 && workspace != nullptr) {
    // Giải QP và chấp nhận cả nghiệm chính xác lẫn gần chính xác.
    osqp_solve(workspace);
    result.status = workspace->info->status;
    result.status_value = workspace->info->status_val;
    result.iterations = workspace->info->iter;
    result.solve_time_ms = 1000.0 * workspace->info->solve_time;
    result.solved = workspace->info->status_val == OSQP_SOLVED ||
      workspace->info->status_val == OSQP_SOLVED_INACCURATE;
    if (result.solved && workspace->solution != nullptr && workspace->solution->x != nullptr) {
      result.predicted_errors.reserve(static_cast<std::size_t>(horizon + 1));
      for (c_int k = 0; k <= horizon; ++k) {
        result.predicted_errors.emplace_back(
          workspace->solution->x[3 * k],
          workspace->solution->x[3 * k + 1],
          workspace->solution->x[3 * k + 2]);
      }
      result.delta_controls.reserve(static_cast<std::size_t>(horizon));
      result.control_sequence.reserve(static_cast<std::size_t>(horizon));
      for (c_int k = 0; k < horizon; ++k) {
        const Eigen::Vector2d delta(
          workspace->solution->x[state_count + 2 * k],
          workspace->solution->x[state_count + 2 * k + 1]);
        const auto & ref = reference[static_cast<std::size_t>(k)];
        result.delta_controls.push_back(delta);
        result.control_sequence.emplace_back(
          ref.linear_velocity + delta[0], ref.angular_velocity + delta[1]);
      }
      result.obstacle_slacks.reserve(static_cast<std::size_t>(slack_count));
      for (c_int slack_index = 0; slack_index < slack_count; ++slack_index) {
        result.obstacle_slacks.push_back(
          workspace->solution->x[slack_start + slack_index]);
      }
      result.control = result.control_sequence.front();
      result.solved = result.control.allFinite();
      for (const auto & error : result.predicted_errors) {
        result.solved = result.solved && error.allFinite();
      }
    }
  } else {
    result.status = std::string(rebuild ? "osqp_setup failed: " : "osqp_update failed: ") +
      std::to_string(workspace_status);
  }
  // Giải phóng hai wrapper CSC được OSQP tạo ra.
  c_free(p);
  c_free(a);
  return result;
}

}  // namespace bumperbot_motion
