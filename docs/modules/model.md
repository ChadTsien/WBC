# t1_model

> 目录已按 horkin-core 重排成仓库根下的同名目录，不再使用 `src/t1_*` 包。文中若还出现旧路径，以 README 的目录表和 `docs/PROGRESS.md` 为准。


Pinocchio 2.7 的封装，提供机械臂运动学、动力学和差速底盘运动学。core 库，命名空间 `T1`。头文件只包含 `pinocchio/multibody/fwd.hpp`，完整的 Pinocchio 头文件只在 `.cpp` 里包含，缩短编译时间。

**测试数据**：单测需要展开后的 URDF。建议在 `test/data/t1.urdf` 放一份由 xacro 生成的文件（`xacro t1.urdf.xacro use_ros2_control:=false > test/data/t1.urdf`），模型改动后重新生成。

**共享 Data 的规则**：`ArmKinematics` 和 `Dynamics` 共用 `RobotModel::data()`。每个周期先 `kinematics.update(q)`，把需要的位姿和雅可比都取走，再调用 `dynamics`。

---

### include/t1_model/robot_model.hpp · src/robot_model.cpp
- **职责**：从 URDF 文本建立"以 base_link 为固定基座的机械臂模型"，锁掉轮子等其他关节；解析坐标系 ID；提供关节限位。
- **实现要点**
  - `pinocchio::urdf::buildModelFromXML(config.urdf_xml, full_model)`。
  - 不在 `arm_joints` 里的关节（驱动轮）用 `pinocchio::buildReducedModel(full, locked_ids, q_ref, model)` 锁掉。continuous 关节的 nq = 2，`q_ref` 用 `pinocchio::neutral(full)`。
  - 检查锁完后 `model.nq == 6`，且关节顺序与 `arm_joints` 一致（`model.getJointId(name)` 依次递增），否则抛异常。
  - `tool_frame`、`ft_frame` 用 `existFrame`/`getFrameId` 解析，不存在就抛异常并带上名字。
  - 限位取自 `model.lowerPositionLimit`、`upperPositionLimit`、`velocityLimit`。
- **测试要点**：加载 `test/data/t1.urdf`：nq == 6；`brush_tip` 和 `ft_sensor` 存在；限位与 `joint_limits.yaml` 一致；传入错误的关节名时抛异常。
- **里程碑** M0　**行数预算** ≤ 120

### include/t1_model/arm_kinematics.hpp · src/arm_kinematics.cpp
- **职责**：正运动学、坐标系位姿、雅可比（base_link 坐标轴）、可操作度。
- **实现要点**
  - `update`：`forwardKinematics` → `updateFramePlacements` → `computeJointJacobians`。
  - `frame_pose`：`data.oMf[id]` 转成 `Iso3`（`translation()` 和 `rotation()`）。
  - `frame_jacobian`：`getFrameJacobian(model, data, id, LOCAL_WORLD_ALIGNED, J)`。这里的 "WORLD" 指 Pinocchio 模型的根，也就是 base_link；前 3 行线速度，后 3 行角速度。
  - `manipulability`：取刷头坐标系的 x、y 轴 `u1`、`u2`（都垂直于刷头轴线），组成 5×6 矩阵 `[Jv; u1ᵀJw; u2ᵀJw]`，返回 `sqrt(det(J Jᵀ))`。
- **测试要点**
  - 数值雅可比：随机 q，每个关节扰动 1e-6，位置差分和 `log3(R0ᵀR1)` 差分，与解析雅可比的误差 < 1e-5。
  - 零位时 `brush_tip` 的位置与手算（或 RViz 读数）一致。
  - 在奇异构型（J5/J6 伸直）附近，`manipulability` 明显变小。
- **里程碑** M0（位姿、雅可比），M1（可操作度）　**行数预算** ≤ 100

### include/t1_model/base_kinematics.hpp · src/base_kinematics.cpp
- **职责**：差速底盘的轮速与 `[v, ω]` 互转；`[v, ω]` 到"固连在底盘上的一点"的速度雅可比。
- **实现要点**
  - `v = r(ωl + ωr)/2`，`ω = r(ωr − ωl)/L`；反解 `ωl = (v − ωL/2)/r`，`ωr = (v + ωL/2)/r`。
  - `point_jacobian(p_b, yaw)`：设 `R = Rz(yaw)`，`p_w = R p_b`（相对底盘原点）。
    - v 列：`[cos yaw, sin yaw, 0, 0, 0, 0]ᵀ`；
    - ω 列：`[ẑ × p_w; 0, 0, 1]`，即 `[−p_w.y, p_w.x, 0, 0, 0, 1]ᵀ`。
- **测试要点**：轮速 → twist → 轮速往返一致；原地转（ωl = −ωr）时 v = 0；`point_jacobian` 与"按差速模型积分一个很小的 dt 后的差分"一致。
- **里程碑** M0（互转），M1（雅可比）　**行数预算** ≤ 60

### include/t1_model/whole_body_jacobian.hpp · src/whole_body_jacobian.cpp
- **职责**：刷头在世界系下的 6 维速度对 `[v, ω, dq1..dq6]` 的 6×8 雅可比。
- **实现要点**
  - 前 2 列：`base.point_jacobian(tool_in_base.translation(), base_pose.yaw)`；
  - 后 6 列：`blockdiag(R, R) · arm_jacobian`，其中 `R = Rz(yaw)`，把 base_link 坐标轴转到世界坐标轴。
- **测试要点**：随机底盘位姿、关节角和输入 x；把状态按 x 推进一个很小的 dt（底盘按差速模型积分，关节 q += dq·dt），用新旧刷头位姿的差分验证 `J·x`，误差 < 1e-5。**这是整个 WBC 最关键的一个测试。**
- **里程碑** M1　**行数预算** ≤ 40

### include/t1_model/dynamics.hpp · src/dynamics.cpp
- **职责**：MIT 前馈用的重力矩；刷头负载并入最后一个连杆。
- **实现要点**
  - 构造时保存 `model.inertias[最后一个关节]` 的原始值。
  - `set_tool_payload`：用 `model.frames[ft_frame].placement` 把负载 `pinocchio::Inertia(mass, com, 质点惯量)` 变换到关节坐标系，再与**原始值**相加（不能在当前值上累加，否则每换一次刷头就重复叠加）。
  - `gravity_torque`：`model.gravity.linear(gravity_in_base)`，然后 `computeGeneralizedGravity(model, data, q)`。
- **测试要点**：水平地面上 J1、J2、J4 的重力矩 ≈ 0；把负载设为 m 后，J6 重力矩的增量 ≈ `m·g·水平力臂`；把重力方向倾斜 10°，J1 的重力矩变为非零。
- **里程碑** M3a（负载），M3b（重力矩用于 MIT 前馈）　**行数预算** ≤ 70

### 计划中（M5 创建）
- `collision_model.hpp/.cpp`：hpp-fcl 碰撞对（简化几何体）、最近距离和最近点，给 `t1_wbc/constraint_collision` 用。
