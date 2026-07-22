# Kế hoạch triển khai MPC Controller cho Bumperbot

## 1. Mục tiêu

Triển khai một controller plugin thứ ba trong `bumperbot_motion`, viết mã tối giản giống pd_motion_planner.cpp và pure_pursuit.cpp:

```text
bumperbot_motion::MPCController
```

Controller chạy trên ROS 2 Humble, Nav2 và Gazebo, điều khiển robot vi sai bằng:

```text
u = [v, omega]
```

Kế hoạch được chia thành các process tăng dần về độ hoàn thiện. Nguyên tắc thực hiện là:

1. Process đầu tiên chỉ nhằm làm thuật toán chạy được trên mô phỏng trống.
2. Chỉ chuyển process sau khi đạt đầy đủ điều kiện hoàn thành của process hiện tại.
3. Không tối ưu sớm khi chưa chứng minh kết quả toán học và hành vi cơ bản là đúng.
4. Mỗi process phải giữ controller ở trạng thái build được và chạy được.

---

## 2. Các quyết định kỹ thuật đã chốt

- Hệ điều hành: Ubuntu 22.04.
- Middleware: ROS 2 Humble.
- Navigation framework: Nav2 Controller Server.
- Môi trường chạy ban đầu: Gazebo simulation.
- Loại controller: plugin kế thừa `nav2_core::Controller`.
- Mô hình robot: unicycle/differential drive.
- Thuật toán: Linear Time-Varying MPC trong hệ tọa độ lỗi của robot.
- Solver: OSQP 0.6.2 từ `ros-humble-osqp-vendor`.
- Đại số tuyến tính: Eigen3.
- Tần số controller ban đầu: 20 Hz, tương ứng `dt = 0.05 s`.
- Horizon ban đầu: `N = 10`; chỉ tăng sau khi đã đo thời gian giải.
- MPC phiên bản đầu chỉ bám đường. Tránh vật cản chủ động là phần nâng cao.

Sai số bám quỹ đạo trong hệ tọa độ robot:

```text
e = [e_x, e_y, e_yaw]
```

Biến điều khiển tối ưu:

```text
delta_u = u - u_reference
```

Hàm mục tiêu:

```text
J = sum(
      e' Q e
      + delta_u' R delta_u
      + delta_u_rate' S delta_u_rate
    )
    + e_terminal' Q_terminal e_terminal
```

---

## 3. Kiến trúc đích

```text
bumperbot_motion/
├── MPC_IMPLEMENTATION_PLAN.md
├── include/bumperbot_motion/
│   ├── mpc_controller.hpp
│   ├── mpc_solver.hpp
│   └── reference_trajectory.hpp
├── src/
│   ├── mpc_controller.cpp
│   ├── mpc_solver.cpp
│   └── reference_trajectory.cpp
├── test/
│   ├── test_mpc_solver.cpp
│   └── test_reference_trajectory.cpp
├── CMakeLists.txt
├── package.xml
└── motion_planner_plugins.xml
```

Trách nhiệm của từng thành phần:

- `MPCController`: giao tiếp ROS/Nav2, lifecycle, TF, path, velocity, costmap và publisher debug.
- `ReferenceTrajectory`: chiếu robot lên path, arc length, nội suy và tạo trajectory tham chiếu.
- `MPCSolver`: dựng QP, gọi OSQP và trả chuỗi điều khiển/trạng thái dự báo.

---

# Process 1 — Minimum Viable MPC: build, load và chạy được

## Mục tiêu

Tạo phiên bản MPC nhỏ nhất có thể build, được Nav2 load và điều khiển robot bám một đường đơn giản trong Gazebo trống. Process này chưa yêu cầu tracking tối ưu, tránh vật cản hay hiệu năng cao.

## Phạm vi

### 1.1. Tích hợp dependency

Cập nhật `package.xml`:

```xml
<depend>osqp_vendor</depend>
<depend>eigen</depend>
```

Cập nhật `CMakeLists.txt`:

```cmake
find_package(Eigen3 REQUIRED)
find_package(osqp_vendor REQUIRED)
```

Liên kết MPC core với:

```cmake
Eigen3::Eigen
osqp::osqp
```

Trước khi viết toàn bộ controller, tạo một bài toán QP rất nhỏ để xác nhận header, target link và OSQP runtime hoạt động.

### 1.2. Tạo plugin skeleton

Tạo `MPCController` triển khai đủ interface:

- `configure()`
- `activate()`
- `deactivate()`
- `cleanup()`
- `setPlan()`
- `computeVelocityCommands()`
- `setSpeedLimit()`

Khai báo plugin trong `motion_planner_plugins.xml` và thêm target vào `CMakeLists.txt`.

Ở bước skeleton, `computeVelocityCommands()` có thể trả zero velocity để xác nhận:

- Plugin được phát hiện.
- Lifecycle chạy đúng.
- Không có lỗi symbol hoặc pluginlib.
- Controller Server gọi được plugin ở 20 Hz.

### 1.3. Tạo reference tối thiểu

Reference phiên bản đầu chỉ cần:

1. Giữ nguyên `global_plan_` nhận từ `setPlan()`.
2. Mỗi chu kỳ tạo một bản path đã transform sang frame của local costmap hoặc `robot_pose`.
3. Tìm segment gần robot nhất.
4. Lấy `N + 1` điểm phía trước theo khoảng cách gần bằng:

```text
delta_s = v_reference * dt
```

5. Nội suy tuyến tính `x`, `y` và yaw của segment.
6. Dùng một `v_reference` nhỏ và cố định, ví dụ `0.15 m/s`.
7. Đặt `omega_reference = v_reference * curvature`; ở bản đầu có thể dùng curvature đơn giản từ ba điểm.

Chưa cần speed profile tối ưu. Chỉ thử với path thẳng hoặc cong nhẹ, robot có hướng ban đầu gần với hướng path.

### 1.4. Tạo MPC solver tối thiểu

QP gồm:

- Trạng thái lỗi `e_0 ... e_N`.
- Điều khiển sai lệch `delta_u_0 ... delta_u_(N-1)`.
- Equality constraints cho động học LTV.
- Box constraints cho `v` và `omega`.
- Rate constraints cho gia tốc tuyến tính và góc.
- Initial state constraint cố định `e_0` bằng sai số hiện tại.

Constraint đầu tiên phải dùng velocity hiện tại do Nav2 truyền vào:

```text
-a * dt <= (delta_u_0 + u_reference_0) - u_current <= a * dt
```

Trong Process 1, được phép dựng lại QP và gọi `osqp_setup()` mỗi chu kỳ để ưu tiên tính đúng và đơn giản. Việc tái sử dụng workspace sẽ làm ở process tối ưu hiệu năng.

### 1.5. Vòng điều khiển tối thiểu

Trong `computeVelocityCommands()`:

1. Kiểm tra plan rỗng.
2. Transform một bản sao của plan.
3. Sinh reference trajectory.
4. Tính sai số ban đầu trong robot frame.
5. Gọi MPC solver.
6. Lấy điều khiển đầu tiên:

```text
u_0 = u_reference_0 + delta_u_0
```

7. Clamp lại output theo giới hạn cứng.
8. Nếu solver lỗi, output NaN hoặc không có plan thì trả zero velocity và log warning.
9. Nếu robot đã rất gần cuối path thì trả zero velocity trong phiên bản đầu.

### 1.6. Cấu hình chạy ban đầu

Giá trị khởi đầu an toàn:

```yaml
FollowPath:
  plugin: "bumperbot_motion::MPCController"
  prediction_horizon: 10
  time_step: 0.05
  reference_linear_velocity: 0.15
  min_linear_velocity: 0.0
  max_linear_velocity: 0.20
  max_angular_velocity: 0.8
  max_linear_acceleration: 0.5
  max_angular_acceleration: 2.0
  q_x: 10.0
  q_y: 20.0
  q_yaw: 5.0
  r_v: 0.5
  r_omega: 0.2
  rd_v: 2.0
  rd_omega: 0.5
```

### 1.7. Kiểm tra smoke test

```bash
colcon build --packages-select bumperbot_motion
source install/setup.bash
colcon test --packages-select bumperbot_motion
colcon test-result --verbose
```

Chạy Gazebo với:

- Không có vật cản.
- Robot gần đúng hướng ban đầu.
- Goal nằm trên đường thẳng trước mặt.
- Sau khi đường thẳng chạy được mới thử đường cong nhẹ.

## Điều kiện hoàn thành Process 1

- [x] `bumperbot_motion` build thành công và không có compiler warning mới nghiêm trọng.
- [x] Nav2 load được `bumperbot_motion::MPCController`.
- [x] Controller lifecycle activate/deactivate/cleanup không crash.
- [x] OSQP trả trạng thái solved cho đường thẳng.
- [x] Robot phát lệnh `v`, `omega` hữu hạn và nằm trong giới hạn.
- [x] Robot chạy theo đường thẳng và đến gần goal trong Gazebo trống.
- [x] Solver failure trả zero velocity thay vì làm node crash.

Không chuyển Process 2 nếu plugin chưa chạy ổn định ở tình huống cơ bản này.

---

# Process 2 — Verify toán học và tính đúng của implementation

## Mục tiêu

Chứng minh MPC đang giải đúng bài toán, đúng dấu, đúng index và đúng constraint trước khi tune hoặc tối ưu hiệu năng.

## Công việc

### 2.1. Unit test cho MPC solver

- Zero error và reference hợp lệ phải cho `delta_u` gần zero.
- Sai số dọc, ngang và yaw phải sinh điều khiển đúng chiều mong đợi.
- Equality dynamics residual phải nhỏ hơn tolerance đặt trước.
- Mọi nghiệm phải thỏa velocity bounds.
- Mọi nghiệm phải thỏa acceleration bounds, đặc biệt tại `k = 0`.
- Terminal cost phải làm giảm sai số cuối horizon.
- Solver infeasible, max-iteration và invalid input phải được xử lý rõ ràng.
- Không được có NaN hoặc infinity trong input/output.

### 2.2. Unit test cho reference

- Path thẳng.
- Path cong.
- Path chỉ có một pose.
- Hai pose trùng nhau.
- Robot nằm trước, giữa và sau path.
- Yaw đi qua biên `-pi/pi`.
- Path tự giao nhau không được làm progress nhảy lùi tùy ý.

### 2.3. Publisher debug

Thêm lifecycle publishers:

```text
mpc/reference_path
mpc/predicted_path
```

Hiển thị cả hai trong RViz và xác nhận:

- Reference bắt đầu gần robot.
- Predicted path liên tục.
- Hướng quay đúng dấu.
- Pose và path cùng frame với local costmap.

### 2.4. Logging chẩn đoán

Log có throttle cho:

- OSQP status.
- Số iteration.
- Solve time.
- Tổng thời gian `computeVelocityCommands()`.
- Initial tracking error.
- Lệnh `v`, `omega` đầu tiên.

## Điều kiện hoàn thành Process 2

- [x] Unit test toán học vượt qua.
- [x] Constraint residual nằm trong tolerance.
- [x] Reference và predicted path hiển thị đúng trên RViz.
- [x] Không còn lỗi sai dấu rõ ràng trên straight và gentle-curve tests.
- [x] Có log đủ để chẩn đoán solver failure.

---

# Process 3 — Hoàn thiện ReferenceTrajectory và goal behavior

## Mục tiêu

Thay reference tối thiểu bằng reference theo thời gian ổn định, xử lý được cua, giảm tốc và goal pose.

## Công việc

### 3.1. Arc-length parameterization hoàn chỉnh

- Tính cumulative arc length khi nhận plan mới.
- Chiếu robot lên segment gần nhất, không chỉ chọn pose gần nhất.
- Giữ progress/index từ chu kỳ trước để tránh nhảy nhánh.
- Chỉ cho phép lùi index trong một cửa sổ nhỏ khi localization dao động.
- Nội suy `x`, `y`, yaw theo `s`.
- Unwrap yaw trước khi nội suy.
- Dùng hướng segment cho tracking yaw và giữ orientation cuối path cho goal yaw.

### 3.2. Curvature và speed profile

- Tính curvature có lọc để tránh spike ở vertex.
- Tạo speed profile theo curvature.
- Tạo braking profile theo khoảng cách còn lại.
- Bảo đảm profile thỏa giới hạn acceleration/deceleration.
- Tính `omega_reference = v_reference * curvature`.
- Nếu `omega_reference` vượt giới hạn, giảm `v_reference` tương ứng.

### 3.3. Goal handling

Sử dụng tolerance từ `goal_checker->getTolerances()` khi có thể.

Tạo state machine:

```text
ROTATE_TO_PATH -> MPC_TRACKING -> GOAL_ALIGN -> STOPPED
```

- `ROTATE_TO_PATH`: xoay trước khi tracking nếu sai số hướng ban đầu quá lớn.
- `MPC_TRACKING`: điều khiển bám đường bằng MPC.
- `GOAL_ALIGN`: giữ vị trí và căn goal yaw.
- `STOPPED`: trả zero velocity khi goal checker xác nhận hoàn thành.

Dùng hysteresis cho ngưỡng vào/ra rotate mode để tránh chuyển trạng thái liên tục.

## Điều kiện hoàn thành Process 3

- [x] Robot bám được đường thẳng, đường cong nhẹ, góc 90 độ và chữ S.
- [x] Robot giảm tốc trước cua và gần goal.
- [x] Robot bắt đầu lệch hướng lớn có thể xoay rồi mới tracking.
- [x] Robot dừng trong XY/yaw tolerance của goal checker.
- [x] State machine không dao động qua lại quanh threshold.

---

# Process 4 — Hoàn thiện constraints và tương thích Nav2

## Mục tiêu

Bảo đảm MPC tuân thủ giới hạn động học, speed filter và lifecycle của Nav2 trong nhiều điều kiện vận hành.

## Công việc

### 4.1. Constraints đầy đủ

- Linear/angular velocity bounds.
- Linear/angular acceleration bounds.
- Bounds tại `k = 0` dùng measured velocity.
- Clamp output cuối như lớp bảo vệ cuối cùng.
- Kiểm tra measured velocity bất thường hoặc timestamp/chu kỳ không hợp lệ.

### 4.2. `setSpeedLimit()`

- Percentage limit.
- Absolute limit.
- `NO_SPEED_LIMIT`: khôi phục giới hạn gốc.
- Speed limit tác động lên cả QP bounds và reference/braking profile.

### 4.3. Parameter validation

Từ chối cấu hình không hợp lệ trong `configure()`:

- `N <= 0`.
- `dt <= 0`.
- Min velocity lớn hơn max velocity.
- Acceleration không dương.
- Weight âm.
- Kích thước tham số không đúng.

## Điều kiện hoàn thành Process 4

- [ ] Không có command vượt giới hạn cấu hình.
- [ ] Acceleration bound đúng cả khi robot đang chạy rồi đổi controller/path.
- [ ] Speed limit động có hiệu lực và reset đúng.
- [ ] Configure thất bại rõ ràng với parameter không hợp lệ.

---

# Process 5 — Safety bằng local costmap

## Mục tiêu

Ngăn MPC xuất lệnh đưa predicted footprint vào vùng va chạm. Process này chỉ bổ sung khả năng dừng an toàn, chưa làm MPC chủ động tránh vật cản.

## Công việc

### 5.1. Footprint collision checking

- Dùng `nav2_costmap_2d::FootprintCollisionChecker`.
- Lấy footprint từ `costmap_ros_`.
- Lock mutex của costmap trong khi đọc.
- Kiểm tra predicted trajectory trong frame của local costmap.
- Nội suy giữa các predicted poses để không bỏ lọt obstacle.
- Cấu hình rõ cách xử lý unknown, inscribed và lethal cost.

### 5.2. Collision policy

Khi predicted path va chạm:

1. Không xuất command MPC nguy hiểm.
2. Trả zero velocity ở phản ứng đầu tiên.
3. Log warning có throttle và publish trạng thái debug.
4. Sau số chu kỳ hoặc timeout cấu hình, báo controller failure để Nav2 thực hiện replan/recovery.

Không tự động xoay tại chỗ nếu chuyển động xoay chưa được kiểm tra va chạm.

### 5.3. Tần số costmap

Đo hành vi với local costmap hiện tại. Nếu 5 Hz phản ứng quá chậm, tăng `update_frequency` lên khoảng 10 Hz trong Gazebo và đánh giá lại tải CPU.

## Điều kiện hoàn thành Process 5

- [ ] Robot dừng trước vật cản tĩnh trên predicted path.
- [ ] Không bỏ lọt vật cản nằm giữa hai predicted poses.
- [ ] Collision kéo dài dẫn tới replan/recovery thay vì zero command vô hạn.
- [ ] Rotate và goal alignment cũng được collision-check.
- [ ] Không deadlock khi đọc costmap.

---

# Process 6 — Tối ưu hiệu năng và độ ổn định solver

## Mục tiêu

Giảm allocation, giảm thời gian giải và bảo đảm controller đáp ứng ổn định ở 20 Hz.

## Công việc

### 6.1. Tái sử dụng OSQP workspace

- Cố định sparsity pattern theo horizon.
- Chỉ gọi `osqp_setup()` khi configure hoặc khi cấu trúc bài toán thay đổi.
- Mỗi chu kỳ cập nhật `q`, `l`, `u` và các giá trị cần thiết trong `A/P`.
- Dùng `osqp_update_lin_cost`, `osqp_update_bounds`, `osqp_update_A` hoặc `osqp_update_P_A`.
- Shift nghiệm chu kỳ trước một bước rồi warm-start.
- Quản lý OSQP workspace bằng RAII.

### 6.2. Giảm chi phí tính toán

- Preallocate Eigen vectors/matrices.
- Tránh tạo publisher/message hoặc container lớn không cần thiết mỗi chu kỳ.
- Chỉ publish debug path khi có subscriber hoặc theo tần số thấp hơn controller.
- Chỉ transform/cắt phần path cần cho local horizon.
- Tránh log mỗi chu kỳ khi controller hoạt động bình thường.

### 6.3. Solver timeout và fallback

- Đặt `max_iter`, tolerance và time limit thích hợp.
- Phân biệt `solved`, `solved inaccurate`, `max iterations`, `infeasible` và lỗi setup.
- Chỉ chấp nhận nghiệm approximate nếu residual nằm trong ngưỡng an toàn.
- Khi solver lỗi, không tái sử dụng mù quáng command cũ.

### 6.4. Tiêu chí timing

Ở controller frequency 20 Hz:

```text
QP solve P95                  < 5 ms
computeVelocityCommands P95  < 20 ms
hard computation timeout     < 40 ms
control period               = 50 ms
```

Đo ít nhất:

- Mean, P95 và maximum solve time.
- Mean, P95 và maximum callback time.
- Số iteration.
- Tỷ lệ solver failure.
- Số deadline miss.

Chỉ tăng horizon từ `N = 10` lên `N = 15` hoặc `N = 20` sau khi timing còn đủ margin.

## Điều kiện hoàn thành Process 6

- [ ] Không setup/free OSQP workspace ở mỗi chu kỳ.
- [ ] Không có memory leak trong chạy dài.
- [ ] Đạt tiêu chí P95 timing trên máy chạy Gazebo.
- [ ] Không có deadline miss kéo dài.
- [ ] Warm start giảm iteration hoặc solve time có thể đo được.

---

# Process 7 — Tuning và so sánh với PD/Pure Pursuit

## Mục tiêu

Tìm bộ tham số tốt và chứng minh MPC đem lại cải thiện có thể đo được, không chỉ đánh giá bằng quan sát.

## Kịch bản đánh giá

1. Đường thẳng.
2. Góc cua 90 độ.
3. Quỹ đạo chữ S.
4. Robot bắt đầu lệch hướng 45, 90 và 135 độ.
5. Goal gần và goal xa.
6. Nhiễu odometry.
7. Thay đổi speed limit trong lúc chạy.
8. Vật cản xuất hiện trên predicted path.
9. Chạy liên tục nhiều goal để kiểm tra lifecycle và rò rỉ trạng thái.

## Chỉ số so sánh

- RMS cross-track error.
- Maximum cross-track error.
- RMS yaw error.
- Thời gian đến goal.
- Sai số pose cuối.
- Tổng variation của `v` và `omega`.
- Peak acceleration.
- Số lần controller failure/recovery.
- CPU usage.
- Solve/callback timing.

## Thứ tự tuning

1. Chốt `dt`, `N` và giới hạn vật lý.
2. Tune `Q`: ưu tiên tracking position/yaw.
3. Tune `R`: sai số so với reference command.
4. Tune `S`: độ mượt command.
5. Tune terminal cost.
6. Tune curvature speed scaling và braking profile.
7. Tune solver tolerance/iteration sau cùng.

Mỗi lần chỉ thay đổi một nhóm tham số và lưu kết quả để so sánh.

## Điều kiện hoàn thành Process 7

- [ ] Có dữ liệu định lượng cho PD, Pure Pursuit và MPC trên cùng path/điều kiện.
- [ ] MPC đạt tracking error và command smoothness mục tiêu.
- [ ] Bộ tham số mặc định ổn định trên toàn bộ kịch bản chính.
- [ ] Có tài liệu ghi lại bộ tham số và trade-off đã chọn.

---

# Process 8 — Các tối ưu nâng cao, thực hiện khi thật sự cần

Process này không thuộc MVP và chỉ thực hiện sau khi Process 1–7 ổn định.

## 8.1. Cost-aware MPC

- Thêm obstacle/costmap cost vào objective.
- Hoặc thêm linearized obstacle-distance constraints.
- Đánh giá lại tính convex và khả năng giữ bài toán dưới dạng QP.
- Có slack variables để tránh infeasible cứng.

## 8.2. Soft constraints

- Thêm slack cho tracking hoặc một số safety margin phù hợp.
- Phạt slack đủ lớn nhưng không làm bài toán mất điều kiện số.
- Không biến hard physical limits thành soft constraint nếu gây mất an toàn.

## 8.3. Model nâng cao

- Bổ sung động học actuator hoặc wheel-speed constraints nếu mô hình unicycle chưa đủ.
- Xem xét delay compensation từ controller đến Gazebo/phần cứng.
- Xem xét nonlinear MPC chỉ khi LTV-MPC đã được benchmark và chứng minh không đáp ứng yêu cầu.

## 8.4. Dynamic parameter tuning

- Parameter callback an toàn thread.
- Chỉ cho thay đổi weight/bound hợp lệ.
- Rebuild solver có kiểm soát nếu horizon thay đổi.

## Điều kiện hoàn thành Process 8

- [ ] Tối ưu nâng cao giải quyết một hạn chế đã được đo và tái hiện rõ ràng.
- [ ] Không làm giảm safety hoặc timing so với bản ổn định trước đó.
- [ ] Có benchmark trước/sau cho từng thay đổi.

---

## 4. Thứ tự thực hiện tóm tắt

```text
Process 1: Build + plugin load + MPC chạy đường đơn giản
    |
Process 2: Verify toán học, constraints và debug visualization
    |
Process 3: Reference trajectory, giảm tốc và goal behavior
    |
Process 4: Nav2 compatibility và dynamic speed limits
    |
Process 5: Collision safety bằng local costmap
    |
Process 6: OSQP workspace reuse, warm start và timing
    |
Process 7: Tuning + benchmark PD/Pure Pursuit/MPC
    |
Process 8: Cost-aware/soft constraints/model nâng cao nếu cần
```

---

## 5. Definition of Done cho phiên bản MPC ổn định

MPC chỉ được coi là hoàn thành khi:

- [ ] Build và test thành công trên ROS 2 Humble.
- [ ] Nav2 load/unload plugin ổn định.
- [ ] Chạy được các path thẳng, cua 90 độ và chữ S trong Gazebo.
- [ ] Tuân thủ velocity/acceleration bounds.
- [ ] Dừng và căn đúng goal tolerance.
- [ ] Predicted path được collision-check.
- [ ] Solver failure có fallback an toàn.
- [ ] Đạt timing requirement ở 20 Hz.
- [ ] Có benchmark định lượng với PD và Pure Pursuit.
- [ ] Không có memory leak hoặc crash trong bài chạy dài.

---

## 6. Nguyên tắc quản lý thay đổi

- Hoàn thành và kiểm thử từng process trước khi triển khai process tiếp theo.
- Mỗi process nên nằm trong một hoặc một nhóm commit nhỏ, có thể review độc lập.
- Không sửa PD và Pure Pursuit nếu không cần thiết cho MPC.
- Không đổi đồng thời controller, costmap và robot dynamics nếu chưa lưu baseline.
- Luôn giữ một cấu hình MPC chậm nhưng ổn định để làm mốc rollback.
- Khi có lỗi, tái hiện bằng test nhỏ nhất trước khi tiếp tục tune.
