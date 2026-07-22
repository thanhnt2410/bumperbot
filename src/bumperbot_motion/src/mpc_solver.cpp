#include "bumperbot_motion/mpc_solver.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/SparseCore>

extern "C" {
#include "osqp.h"
}

namespace bumperbot_motion
{
// Dựng bài toán QP, gọi OSQP và trả về điều khiển đầu tiên [v, omega].
MPCResult MPCSolver::solve(
  const Eigen::Vector3d & initial_error, const std::vector<ReferencePoint> & reference,
  const Eigen::Vector2d & current_velocity, double dt, const MPCWeights & weights,
  const MPCLimits & limits, const MPCSolverSettings & solver_settings) const
{
  MPCResult result;

  // Kiểm tra dữ liệu đầu vào trước khi dựng ma trận.
  if (reference.size() < 2U || dt <= 0.0 || !initial_error.allFinite() ||
    !current_velocity.allFinite() || solver_settings.max_iterations <= 0 ||
    !weights.state.allFinite() || !weights.terminal.allFinite() ||
    !weights.control.allFinite() || !weights.control_rate.allFinite())
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
  // Biến tối ưu gồm e_0...e_N và delta_u_0...delta_u_(N-1).
  const c_int horizon = static_cast<c_int>(reference.size() - 1U);
  const c_int state_count = 3 * (horizon + 1);
  const c_int variable_count = state_count + 2 * horizon;
  const c_int equality_count = 3 * (horizon + 1);
  const c_int control_bound_count = 2 * horizon;
  const c_int rate_count = 2 * horizon;
  const c_int constraint_count = equality_count + control_bound_count + rate_count;
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
        if (dynamics(i, j) != 0.0) {
          a_entries.emplace_back(row + i, 3 * k + j, -dynamics(i, j));
        }
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
  // Process 1 dựng workspace mới ở mỗi chu kỳ để code đơn giản.
  osqp_set_default_settings(&settings);
  settings.verbose = false;
  settings.warm_start = true;
  settings.max_iter = solver_settings.max_iterations;
  settings.eps_abs = solver_settings.absolute_tolerance;
  settings.eps_rel = solver_settings.relative_tolerance;
  OSQPWorkspace * workspace = nullptr;
  const c_int setup_status = osqp_setup(&workspace, &data, &settings);
  if (setup_status == 0 && workspace != nullptr) {
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
      result.control = result.control_sequence.front();
      result.solved = result.control.allFinite();
      for (const auto & error : result.predicted_errors) {
        result.solved = result.solved && error.allFinite();
      }
    }
    osqp_cleanup(workspace);
  } else {
    result.status = "osqp_setup failed: " + std::to_string(setup_status);
  }
  // Giải phóng hai wrapper CSC được OSQP tạo ra.
  c_free(p);
  c_free(a);
  return result;
}

}  // namespace bumperbot_motion
