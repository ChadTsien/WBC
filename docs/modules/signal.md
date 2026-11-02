# t1_ft

> 目录已按 horkin-core 重排成仓库根下的同名目录，不再使用 `src/t1_*` 包。文中若还出现旧路径，以 README 的目录表和 `docs/PROGRESS.md` 为准。


六维力传感器：重力与零偏辨识、实时补偿、接触与脱落检测、刷头参数管理。命名空间 `T1::ft`。

- `core/`：实时补偿链由 `WbcRosController` 在 500 Hz 回路里调用。
- `ros/`：离线标定节点 `ft_calibration`，不在实时回路中。

**读数模型**（全部在 ft_sensor 系）：

\[
F_s = F_{ext} + m\,g_s + F_b,\qquad \tau_s = \tau_{ext} + c \times (m\,g_s) + \tau_b
\]

其中 \(g_s\) 是重力向量在传感器系中的表示（m/s²），c 是质心，\(F_b\)、\(\tau_b\) 是零偏。共 10 个未知数：m、mc（3）、\(F_b\)（3）、\(\tau_b\)（3）。

**这台机械臂的特点**：J1、J2、J4 都绕竖直轴转动，只有 J5/J6 的俯仰会改变重力在传感器系里的方向。所以标定位姿必须在俯仰方向展开超过 90°，否则矩阵病态。

---

### include/t1_ft/core/gravity_model.hpp · src/core/gravity_model.cpp
- **职责**：前向模型和去除函数。
- **实现要点**：`gravity_wrench`：`f = m·g_s`，`τ = c × f`。`remove_gravity_and_bias`：`raw − gravity_wrench − bias`，力和力矩分别相减。
- **测试要点**：先 `gravity_wrench + bias` 构造一个读数，再去除，结果为 0；m = 0 时只去零偏。
- **里程碑** M3a　**行数预算** ≤ 30

### include/t1_ft/core/gravity_identifier.hpp · src/core/gravity_identifier.cpp
- **职责**：从多个静止位姿的样本中求 10 个参数。
- **实现要点**
  - 未知数 `θ = [m, mc(3), Fb(3), τb(3)]`。
  - 每个样本叠 6 行：
    - 力：`[g_s, 0₃ₓ₃, I₃, 0₃ₓ₃] θ = F_s`；
    - 力矩：`c × (m g) = (mc) × g = −skew(g)·(mc)`，所以是 `[0₃ₓ₁, −skew(g_s), 0₃ₓ₃, I₃] θ = τ_s`。
  - 用 `A.jacobiSvd(ComputeThinU | ComputeThinV)` 求解，`condition = σmax / σmin`；超过 `max_condition` 或样本少于 3 个时 `ok = false`。
  - 解出后 `com = mc / m`（m 太小时报失败）；`rms_force`、`rms_torque` 用残差计算。
  - `solve_bias`：m、c 用已知值，只解零偏：`Fb = mean(F_s − m g_s)`，`τb` 同理。
- **测试要点**
  - 用已知参数加 0.05 N 噪声生成 8 个"只有俯仰变化"的样本（俯仰角在 −60°..+60° 之间）：m 误差 < 2%，c 误差 < 3 mm。
  - 所有样本同一个姿态：`ok = false`，condition 很大。
  - `solve_bias` 只用 1 个样本就能恢复零偏。
- **里程碑** M3a　**行数预算** ≤ 80

### include/t1_ft/core/inertia_compensator.hpp · src/core/inertia_compensator.cpp
- **职责**：去除刷头加速时的惯性力（默认关闭，`inertia_enabled`）。
- **实现要点**：传感器读到的是"为了让刷头加速而施加的力"，相当于读数多了 `m·a`，外力少了 `−m·a`。按约定 `remove_inertia` 返回 `wrench` 力减去 `m·a`、力矩减去 `c × (m·a)`。**符号必须在仿真里验证**：空中让刷头做正弦往复，开启补偿后外力的残余应明显变小，如果变大说明符号反了。转动惯量忽略。
- **测试要点**：a = 0 时结果不变；m = 1、a = (1, 0, 0) 时力的 x 分量变化 1 N。
- **里程碑** M3a　**行数预算** ≤ 20

### include/t1_ft/core/ft_compensator.hpp · src/core/ft_compensator.cpp
- **职责**：实时补偿链：去除零偏和重力 → 去除惯性（可选）→ 低通 → 外力（ft_sensor 系）。
- **实现要点**
  - `configure`：每个滤波器按 `sample_hz`、`cutoff_hz` 配置。
  - `update`：
    - 若在标零中：把"去除重力后的读数 + 当前零偏"（即去除重力但不去零偏的值）累加；累计满 `tare_samples` 个后，求平均作为新零偏，`taring_ = false`。
    - 标零期间输出 0。
  - 第一次调用时用当前值 `reset` 滤波器。
  - 不分配内存。
- **测试要点**
  - 恒定读数 = 重力 + 零偏：输出收敛到 0。
  - 改变零偏后 `request_tare`，`tare_samples` 个周期后输出回到 0，`tare_done()` 为 true。
  - 叠加 5 N 阶跃外力：输出在约 3/(2π fc) 秒内到达 5 N。
- **里程碑** M3a　**行数预算** ≤ 70

### include/t1_ft/core/brush_registry.hpp · src/core/brush_registry.cpp
- **职责**：按刷头 ID 读写重力参数（格式见 `config/brushes.yaml`）。`get`/`set`/`ids` 已写好。
- **实现要点**：`load` 用 `YAML::LoadFile` 读每个刷头的 mass、com、bias_force、bias_torque；`save` 用 `YAML::Emitter` 写回，先写到 `path + ".tmp"` 再 `std::rename`，避免写到一半断电损坏文件。出错时写 `error` 并返回 false。
- **注意**：colcon 安装后 share 目录可能是只读的，或会被下次编译覆盖。实机上把 `brushes_yaml` 参数指向 `~/.config/t1/brushes.yaml`。
- **测试要点**：save 后 load 得到相同数据；文件不存在时 load 返回 false 且 `error` 非空。
- **里程碑** M3a　**行数预算** ≤ 70

### include/t1_ft/core/contact_detector.hpp · src/core/contact_detector.cpp
- **职责**：判断是否接触（带回差）、刷头是否脱落。
- **实现要点**
  - `normal_force = ext·n`：约定 ext 是"环境作用在刷头上的力"，n 指向表面外侧，压墙时墙把刷头往外推，ext 沿 +n，结果为正。传感器读数是哪个方向的力取决于手册定义，**第一次在仿真中压墙时确认符号**，并写成注释。
  - 接触：未接触时 `normal_force > contact_on_force` 进入接触；接触中 `< contact_off_force` 退出。
  - 脱落：补偿仍在扣除刷头的重量，所以掉了以后外力 ≈ `+m·9.81·ẑ`（向上）。条件为"未接触，且 `‖ext − m·9.81·ẑ‖ < detach_tolerance · m·9.81`"，持续 `detach_hold_time` 后置位。
- **测试要点**：力在两个阈值之间来回波动时状态不抖；构造脱落信号，0.1 s 后才报脱落；接触中出现向上的力不报脱落。
- **里程碑** M3b　**行数预算** ≤ 50

### include/t1_ft/ros/ft_calibration_node.hpp · src/ros/ft_calibration_node.cpp · src/ros/ft_calibration_main.cpp
- **职责**：服务 `ft_calibration/calibrate`：发送俯仰位姿参考 → 等待稳定 → 平均读数 → 辨识 → 保存 → 调用 `wbc_controller/set_brush` 让控制器重新加载。
- **实现要点**
  - 两个回调组：服务回调在 `service_group_`（可以阻塞等待）；订阅在 `io_group_`；main 中用 MultiThreadedExecutor（已写好）。
  - `make_poses`：保持当前刷头位置，俯仰角在 `±pitch_span_deg/2` 内均匀取 `count` 个；参考的 `contact_phase = NONE`，`base_mode = TRACK_POSE`（底盘不动）。
  - `move_and_settle`：发一条单点轨迹，等 status 中 ee 位置误差 < `settle_error_m` 并持续 `settle_time_s`，超时返回 false。
  - `collect_sample`：清空窗口，等 `sample_window_s`，平均原始读数；`gravity_in_ft` 由最近一次 IMU 姿态求出 base 系重力，再用 tf 的 `base_link → ft_sensor` 旋转过去。
  - 服务回调结束前恢复标定前的位姿。
- **验收**：仿真中对 sim.yaml 的零偏标定，参数误差满足上面辨识测试的标准；控制器的 ft_status 随之更新。
- **里程碑** M3a　**行数预算** ≤ 200
