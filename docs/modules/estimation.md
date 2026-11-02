# t1_estimation

> 目录已按 horkin-core 重排成仓库根下的同名目录，不再使用 `src/t1_*` 包。文中若还出现旧路径，以 README 的目录表和 `docs/PROGRESS.md` 为准。


底盘状态估计，纯 core 库，命名空间 `T1::est`。由 `WbcRosController` 在 500 Hz 回路里调用，不单独起节点。

**坐标系约定**：EKF 只在 odom 系中工作，状态连续、不跳变。地图定位（Nav2/AMCL）只发布 `map → odom`，不进入 EKF，否则定位跳变会让机械臂猛甩。

---

### include/t1_estimation/diff_drive_odometry.hpp · src/diff_drive_odometry.cpp
- **职责**：把轮子编码器的累计角度积分成 odom 系位姿，同时给出当前 `[v, ω]`。
- **实现要点**
  - 用位置差而不是速度：`dl = pl − pl_last`，`dr = pr − pr_last`；然后 `to_twist({dl, dr})` 得到"一个周期内的位移 ds 和转角 dθ"，再除以 dt 得到速度。
  - 中点积分：`x += ds·cos(yaw + dθ/2)`，`y += ds·sin(yaw + dθ/2)`，`yaw = wrap_angle(yaw + dθ)`。
  - `reset` 已写好。
- **测试要点**：两轮同角度 → 直线前进 `r·θ`；两轮反向 → 原地转 `2rθ/L`；以恒定 v、ω 走一圈回到起点（误差 < 1e-6）。
- **里程碑** M2　**行数预算** ≤ 40

### include/t1_estimation/planar_ekf.hpp · src/planar_ekf.cpp
- **职责**：状态 `[x, y, yaw, v, ω]`，融合轮速里程计的 `[v, ω]` 和陀螺仪 z 轴角速度。
- **实现要点**
  - 预测（恒速模型）：`x += v cos(yaw) dt`，`y += v sin(yaw) dt`，`yaw += ω dt`；雅可比 F 在 (0,2)、(0,3)、(1,2)、(1,3)、(2,4) 处非零；`Q = diag(0, 0, 0, q_v·dt, q_omega·dt)`。
  - 里程计更新：H 选出 v、ω 两维，`R = diag(r_v, r_omega_odom)`。
  - 陀螺仪更新：H 选出 ω 一维，`R = r_omega_gyro`。
  - 门限：新息 y、`S = H P Hᵀ + R`，若 `yᵀ S⁻¹ y > gate_chi2` 则拒绝并返回 false（打滑时里程计会被拒，陀螺仪仍然有效）。
  - 协方差更新用 Joseph 形式 `(I − KH) P (I − KH)ᵀ + K R Kᵀ`，保持对称正定。
  - 全部使用固定大小矩阵，不分配内存。
- **测试要点**
  - 静止、无噪声：状态不变。
  - 用仿真噪声参数喂 10 s 的直线运动：v 的估计误差 < 0.01 m/s。
  - 注入单侧打滑（里程计 ω 突然多出 1 rad/s）：该次 `update_odometry` 返回 false，yaw 误差保持很小。
  - P 始终对称且特征值为正。
- **里程碑** M2　**行数预算** ≤ 100

### include/t1_estimation/tilt.hpp · src/tilt.cpp
- **职责**：由 IMU 姿态求重力在 base_link 中的表示。给机械臂重力补偿和力传感器重力补偿用，斜坡上也正确。
- **实现要点**：`g_world = (0, 0, −g)`；IMU 姿态 `q` 是"IMU 系在重力对齐系中的朝向"，所以 `g_imu = q⁻¹ · g_world`，`g_base = base_R_imu · g_imu`。yaw 对结果没有影响。
- **测试要点**：单位四元数 → `(0, 0, −9.81)`；绕 x 轴转 +20°（车体右倾），重力在 base 系出现 −y 分量（请先画图确认符号）；绕 z 轴任意转动，结果不变。
- **里程碑** M2　**行数预算** ≤ 20
