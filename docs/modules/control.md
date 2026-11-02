# t1_wbc

> 目录已按 horkin-core 重排成仓库根下的同名目录，不再使用 `src/t1_*` 包。文中若还出现旧路径，以 README 的目录表和 `docs/PROGRESS.md` 为准。


全身控制：一个速度级 QP 同时求底盘 `[v, ω]` 和 6 个关节速度，输出 MIT 指令和轮速。命名空间 `T1::wbc`。

- `core/`：纯算法，不依赖 ROS，构造后不再分配内存。
- `ros/`：ros2_control 控制器插件 `t1_wbc/WbcRosController`，它是 500 Hz 回路里唯一的计算者，依次调用估计、力补偿、接触检测和 WBC。

**启用 ProxSuite**：写 `qp_solver.cpp` 时取消 `CMakeLists.txt` 中 `include(T1ProxSuite)` 和链接行的注释。

## QP 形式

决策变量 `x = [v, ω, dq1..dq6]`（8 维）。用 `[v, ω]` 而不是 `[ẋ, ẏ, ω]`，非完整约束天然满足。

\[
\min_x \sum_i w_i \lVert A_i x - b_i \rVert^2 \quad \text{s.t.}\quad x_{lo} \le x \le x_{hi},\; c_{lo} \le Cx \le c_{hi}
\]

| 项 | 维度 | 作用 |
|---|---|---|
| 末端跟踪 | 5 | 刷头位置 + 轴线指向，权重最高 |
| 姿态 | 6 | 让机械臂回到舒适构型；正因为有它，目标走远时底盘才会跟上 |
| 底盘 | 2 | FREE：小正则（底盘是慢自由度）；TRACK_*：跟踪 |
| 限位 | 盒约束 | 速度、加速度、位置 |
| 碰撞（M5） | 不等式 | 速度阻尼器 |

`add_cost` 展开：`H += 2w AᵀA`，`g −= 2w Aᵀb`；最后对角线加 1e-6 正则，保证 H 正定。

---

### include/t1_wbc/core/wbc_types.hpp
- **职责**：枚举、`Reference`（对应 TaskSpacePoint）、`Trajectory`、`RobotState`、`Output`。**状态**：完成。

### include/t1_wbc/core/reference_sampler.hpp · src/core/reference_sampler.cpp
- **职责**：按时间在轨迹上插值出当前参考。
- **实现要点**：`std::upper_bound` 找区间（不分配）；位置、速度、力线性插值；姿态 `Quat::slerp`；底盘 yaw 用 `wrap_angle` 后插值；离散字段取前一个点。超出最后一个点时保持最后位姿，速度清零。
- **测试要点**：两点中点处插值正确；t < 0 返回第一个点；t 超出后速度为 0；yaw 从 3.1 到 −3.1 时走短路径。
- **里程碑** M1　**行数预算** ≤ 60

### include/t1_wbc/core/qp_problem.hpp · src/core/qp_problem.cpp
- **职责**：累积代价和约束，存储在构造时一次性分配。
- **实现要点**：`reset` 把 H、g 清零，边界设为 ±∞，`used_rows_ = 0`；两个 `add_cost` 重载（按行加权就是 `Aᵀ diag(w) A`）；`add_inequality` 写到 `C_.middleRows(used_rows_, n)`；`intersect_bounds` 用 `cwiseMax`/`cwiseMin`。
- **测试要点**：单个代价 `‖x − b‖²` 的解析最优 `−H⁻¹g == b`；超出最大行数时返回 false 且不写入；两次 `intersect_bounds` 取交集。
- **里程碑** M1　**行数预算** ≤ 60

### include/t1_wbc/core/qp_solver.hpp · src/core/qp_solver.cpp
- **职责**：ProxSuite 稠密 QP 的封装，ProxSuite 类型不出现在头文件里。
- **实现要点**
  - `Impl` 持有 `proxsuite::proxqp::dense::QP<double> qp(8, 0, max_rows, true)`（最后一个参数 true 表示启用盒约束）。
  - C 始终传满 `max_rows` 行：未使用的行置 0，上下界设为 ±1e20。这样维度固定，可以只调 `update` 而不重新 `init`。
  - 第一次 `init`，之后 `update`；`warm_start` 时把上一次的解作为初值。
  - 根据 `qp.results.info.status` 映射到 `QpStatus`，`solve_time_us` 取 `info.run_time`。
  - 失败时不改 `*x`。
- **测试要点**：无约束时与 `−H⁻¹g` 一致；盒约束生效（解落在边界上）；不可行问题（下界 > 上界）返回 kInfeasible 且 x 不变；在本机上 1000 次求解的 P99 < 200 µs。
- **里程碑** M1　**行数预算** ≤ 90

### include/t1_wbc/core/task_ee_tracking.hpp · src/core/task_ee_tracking.cpp
- **职责**：刷头 5 维跟踪：位置 3 + 垂直于刷头轴线的 2 个角速度分量（绕轴线转动不约束）。
- **实现要点**
  - 位置误差：`e_p = p_ref − p`。
  - 轴线误差：设 `a = R.col(2)`、`a_ref = R_ref.col(2)`，`e_w = a × a_ref`；投影到刷头 x、y 轴：`e_ax = R.col(0)·e_w`，`e_ay = R.col(1)·e_w`。
  - A 的 5 行：J 的前 3 行；`R.col(0)ᵀ J_ω`；`R.col(1)ᵀ J_ω`。
  - `b = v_ref + kp ⊙ e`，其中 kp 项先用 `saturate_norm` 限幅到 `max_error_rate`（位置、角度分别限）。参考角速度同样投影。
  - 用按行加权的 `add_cost`。
- **测试要点**：零误差、零参考速度时 b = 0；纯绕轴线的旋转误差 → 轴线误差为 0；轴线偏 0.1 rad → 误差范数 ≈ 0.1。
- **里程碑** M1　**行数预算** ≤ 70

### include/t1_wbc/core/task_posture.hpp · src/core/task_posture.cpp
- **职责**：把机械臂拉向 `q_nominal`。
- **实现要点**：`A = [0₆ₓ₂ | I₆]`，`b = kp (q_nominal − q)`，按行权重 `weight`。
- **测试要点**：只有这一个任务时，解的关节部分等于 b，底盘部分为 0（需另加正则）。
- **里程碑** M1　**行数预算** ≤ 30

### include/t1_wbc/core/task_base.hpp · src/core/task_base.cpp
- **职责**：按模式给底盘加代价。
- **实现要点**
  - kFree：`A = [I₂ | 0]`，`b = 0`，权重 `regularization`。
  - kTrackTwist：`b = [v, ω]_ref`，权重 `track_weight`。
  - kTrackPose：把世界系位置误差转到底盘系 `e_b = Rz(−yaw)(p_ref − p)`；`v = kp_position · e_b.x`，`ω = kp_yaw · wrap_angle(yaw_ref − yaw)`。横向误差 `e_b.y` 无法直接纠正，靠参考轨迹本身可行。然后按 TrackTwist 处理，并叠加参考速度作为前馈。
- **测试要点**：目标在正前方 1 m → v > 0、ω = 0；目标朝向差 90° → ω > 0；kFree 时 b = 0。
- **里程碑** M1　**行数预算** ≤ 50

### include/t1_wbc/core/constraint_limits.hpp · src/core/constraint_limits.cpp
- **职责**：把所有限位写成 x 的盒约束。
- **实现要点**
  - 速度：`|dq| ≤ dq_max`，`|v| ≤ v_max`，`|ω| ≤ omega_max`。
  - 加速度：`x_prev ± a_max · dt`。
  - 位置：`dq ≤ position_gain (q_max − margin − q)`，`dq ≥ position_gain (q_min + margin − q)`。
  - 三者取交集。若交集为空（例如关节已超出限位），把位置约束放宽到"允许回到限位内"的方向，保证可行。
- **测试要点**：q 接近 q_max 时上界趋于 0；q 已超出 q_max 时上界为负（必须往回走）且下界 ≤ 上界；加速度约束以 x_prev 为中心。
- **里程碑** M1　**行数预算** ≤ 50

### include/t1_wbc/core/contact_phase.hpp · src/core/contact_phase.cpp
- **职责**：把任务层请求的阶段和测得的接触状态，转成法向上的力或速度参考。
- **实现要点**
  - kNone：不用导纳，力参考归零。
  - kApproach：沿 −n 以 `approach_speed` 前进；一旦 `in_contact`，开始用导纳，力参考从测量值起斜坡升到期望值。
  - kClean：`force_ref` 以 `force_ramp_rate` 斜坡逼近 `desired_normal_force`（夹到 `max_force`），使用导纳和柔顺增益。
  - kRetract：力参考斜坡降到 0；测量力 < `released_force` 后沿 +n 以 `retract_speed` 抬起。
  - 用 `RateLimiter`。
- **测试要点**：每个阶段的输出符合描述；从 kClean 切到 kRetract 时力参考连续下降，没有跳变。
- **里程碑** M3b　**行数预算** ≤ 70

### include/t1_wbc/core/admittance.hpp · src/core/admittance.cpp
- **职责**：法向导纳（力跟踪）和倾斜导纳（让刷头贴平）。
- **实现要点**
  - 法向：`a = (F_meas − F_des − D v − K x) / M`；半隐式欧拉 `v += a dt`（夹到 `max_velocity`），`x += v dt`（夹到 `max_offset`，夹住时 v 清零）；返回 `offset = x · n`。
  - 符号：x 沿 +n 为正。压力过大时 `F_meas > F_des`，x 增大，刷头后退。
  - 倾斜：把接触力矩中垂直于刷头轴线的分量 `τ⊥ = τ − (τ·a)a` 积分为 `tilt += τ⊥ / tilt_damping · dt`，范数夹到 `max_tilt`。
- **测试要点**：K = 0 时，对一个虚拟弹簧墙（F = k·穿透量）闭环仿真，F 收敛到 F_des 且无振荡；`reset` 后 offset 为 0；offset 不超过上限。
- **里程碑** M3b　**行数预算** ≤ 60

### include/t1_wbc/core/command_builder.hpp · src/core/command_builder.cpp
- **职责**：QP 速度解 → MIT 指令。
- **实现要点**：`q_des += dq · dt`（从上一次 q_des 积分，不从测量值），再夹到 `[q_meas ± max_tracking_error]` 和 `[q_min, q_max]`；`dq_des = dq`；`tau_ff = tau_gravity`；kp/kd 按 `contact_gains` 在两组之间选择。
- **测试要点**：dq 恒定时 q_des 线性增长；测量值卡住不动时 q_des 不会离开它超过 `max_tracking_error`；`reset` 后 q_des = q_meas。
- **里程碑** M1　**行数预算** ≤ 40

### include/t1_wbc/core/wbc_config.hpp · src/core/wbc_config.cpp
- **职责**：`WbcConfig` 聚合所有模块配置；`load_wbc_config` 从 `config/wbc.yaml` 读取。
- **实现要点**：每个小节一个静态函数（`load_ee`、`load_limits` …），保证单个函数不超过 50 行；用 `require`/`get_or`/`require_vector`。
- **测试要点**：读取仓库里的 `wbc.yaml` 不抛异常，抽查几个字段；删掉一个必需键时抛异常且信息包含键名。
- **里程碑** M1　**行数预算** ≤ 150

### include/t1_wbc/core/wbc_controller.hpp · src/core/wbc_controller.cpp
- **职责**：一个控制周期的全部计算。构造和 `reset` 已写好。
- **`update` 步骤**
  1. `kinematics_.update(q)`，取 `tool_in_base`、刷头雅可比、可操作度。
  2. `ee_world = base_pose ⊗ tool_in_base`。
  3. `contact_.update(...)` 得到法向的力或速度参考。
  4. 若使用导纳：`admittance_.update_normal/update_tilt`，修正参考的刷头位置和轴线；接近或抬起阶段改为修正参考速度。
  5. `whole_body_jacobian(...)`。
  6. `qp_.reset()`，依次加入末端、姿态、底盘任务，`limits_.apply`。
  7. `solver_.solve`；失败时 x = 0（保持不动），并记录状态。
  8. `dynamics_.gravity_torque(q, gravity_in_base)`。
  9. `command_.build(...)`，`base_kinematics_.to_wheels(...)`；更新 `x_prev_`，填写 Output。
- **验收（M1 闭环运动学测试）**：写一个不依赖 ROS 的测试：用"理想执行"（q += dq·dt，底盘按差速模型积分）闭环 10 s，参考为匀速移动的刷头目标，刷头误差 < 5 mm；目标走到机械臂够不着的地方时底盘自动跟随。
- **里程碑** M1（M3b 加接触）　**行数预算** ≤ 150

### include/t1_wbc/ros/wbc_ros_io.hpp · src/ros/wbc_ros_io.cpp
- **职责**：控制器所有面向 ROS 的部分：订阅、服务、实时发布者。
- **实现要点**
  - `on_reference`：把消息转成 `StampedTrajectory`（在非实时线程里分配），`writeFromNonRT`。
  - `on_cmd_vel`：记录 twist 和接收时间；实时侧 `teleop(now)` 超过 `teleop_timeout_s` 返回空。
  - `on_tare`：置原子标志；`take_tare_request` 用 `exchange(false)`。
  - `on_set_brush`：在服务线程里从 registry 加载（文件 I/O 不进实时线程），连同递增的序号写入 buffer；实时侧比较序号，只处理一次。
  - `publish`：
    - status 按 `status_rate_hz` 降频；
    - 每个发布者 `trylock()` 成功才填数据并 `unlockAndPublish()`；
    - odom 和 `odom → base_link` 的 tf 由这里发布。
- **测试要点**：用 launch_testing 或手动：发参考后 status 中的误差变化；断开 cmd_vel 0.5 s 后底盘停止。
- **里程碑** M1（M3 加力传感器相关）　**行数预算** ≤ 250（超出时把消息转换拆到 `msg_conversions.cpp`）

### include/t1_wbc/ros/wbc_ros_controller.hpp · src/ros/wbc_ros_controller.cpp
- **职责**：`t1_wbc/WbcRosController` 插件。
- **实现要点**
  - `on_init`：声明参数 `arm_joints`、`wheel_joints`、`imu_name`、`ft_name`、`wbc_config`、`robot_description_node`。
  - `*_interface_configuration`：按固定顺序列出接口名：
    - 命令：每个关节 5 个接口，然后每个轮子 1 个；
    - 状态：每个关节位置、速度、力矩，每个轮子位置、速度，IMU 10 个，力传感器 6 个。
    - 下标辅助函数依赖这个顺序。
  - `on_configure`：`load_wbc_config`，`fetch_robot_description` 后构造 `RobotModel`，接着构造 WBC、里程计、EKF、力补偿和 io。所有分配都在这里完成。
  - `on_activate`：读当前状态，`odometry_->reset`，`ekf_.reset`，`wbc_->reset`；`hold_reference_` 设为当前刷头位姿。
  - `update`：
    1. 处理换刷头（`set_tool_payload` 和补偿器参数）和标零请求；
    2. `read_state`：里程计 → EKF（预测、里程计更新、陀螺仪更新）→ 倾角 → 力补偿 → 转到世界系 → 接触检测；
    3. `current_reference`：有轨迹就按时间采样，有遥控就覆盖底盘模式，都没有就用 `hold_reference_`；
    4. `wbc_->update`；
    5. 刷头脱落或 QP 连续失败时 `write_hold`，否则 `write_command`；
    6. `io_.publish`。
- **验收**：`ros2 launch t1_bringup sim.launch.py` 后向 `/wbc_controller/cmd_vel` 发遥控速度，刷头在世界系中保持不动（M1 验收）。
- **里程碑** M1　**行数预算** ≤ 250

### config/wbc.yaml
- **职责**：所有模块参数的初值，调参只改这里。

### 计划中
- `core/constraint_collision.hpp/.cpp`（M5）：用 `t1_model/collision_model` 的距离 d 和雅可比，加入速度阻尼器（velocity damper）约束 `J_d x ≥ −ξ (d − d_s)/(d_i − d_s)`。
- `core/safety_monitor.hpp/.cpp`（M5）：人靠近降速、力超限停机、QP 连续失败停机。
