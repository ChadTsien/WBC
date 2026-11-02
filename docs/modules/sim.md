# t1_sim

> 目录已按 horkin-core 重排成仓库根下的同名目录，不再使用 `src/t1_*` 包。文中若还出现旧路径，以 README 的目录表和 `docs/PROGRESS.md` 为准。


MuJoCo 仿真，以 ros2_control 硬件插件的形式接入。控制器看到的接口与实机完全相同，从仿真切到实机只需改 `use_sim`。命名空间 `T1::sim`。

- `core/`：只依赖 MuJoCo 和 t1_common，不依赖 ROS。
- `ros/`：`T1SimSystem` 插件。

**启用 MuJoCo**：开始写 `mujoco_world.cpp` 时，取消 `CMakeLists.txt` 中 `include(T1MuJoCo)` 和 `target_link_libraries(... T1::mujoco)` 的注释（需要先 `sudo ./scripts/setup_deps.sh` 装好 MuJoCo 3.9.0 C 库）。

**时间关系**：controller_manager 每 2 ms 调用一次 `read → update → write`；`read()` 中推进 `period / timestep` 步物理（默认 2 步），每一步都按最新指令重新计算 MIT 力矩。仿真时间因此与控制周期绑定。

---

### include/t1_sim/core/mujoco_world.hpp · src/core/mujoco_world.cpp
- **职责**：持有 `mjModel`/`mjData`；所有 MuJoCo API 调用只出现在这个文件里。
- **实现要点**
  - `Impl` 里存模型指针、数据指针、`std::mutex`（给 viewer 用），以及各 id：6 个关节的 `qpos`/`qvel` 地址、6 个 motor 执行器、2 个轮子执行器、IMU 与力传感器的 sensor 地址、`brush_tip` 的 site、底盘 body。
  - 构造：`mj_loadXML(path, nullptr, err, sizeof err)`，失败时抛异常并带上 err；用 `mj_name2id` 按名字查 id，名字和 `NAME_MAPPING.md` 保持一致，找不到就抛异常。
  - `step()`：
    - 每个关节算 `τ = kp(q_des − q) + kd(dq_des − dq) + τ_ff`，夹到 `±effort_limits` 后写入 `ctrl`；
    - 轮速写入 velocity 执行器；
    - 然后 `mj_step`，整个过程加锁。
  - 传感器读取用 `sensordata[adr..adr+n)`。MuJoCo 的 `force`/`torque` 传感器给出的是"子体作用在父体上"的力，符号和坐标系要对照力传感器手册定义，必要时取反。
  - `imu_linear_acceleration` 用 `accelerometer` 传感器（含重力反作用，与真实 IMU 一致）。
  - `ground_truth`：从底盘的 freejoint `qpos` 解算 x、y、yaw，从 site 的 `xpos`/`xmat` 得到刷头位姿。
- **测试要点**（需要生成好的 MJCF）
  - 零力矩下机械臂受重力下垂，底盘静止不动。
  - kp = 20、kd = 1、q_des = 0.3 时，J1 在 1 s 内收敛到 0.3。
  - 两轮同速时，`ground_truth` 的 yaw 不变、x 增加。
  - 静止时 `ft_raw` 的力 ≈ 刷头重量（方向随姿态变化）。
- **里程碑** M0　**行数预算** ≤ 220（超出时把 id 查找拆到 `mujoco_ids.cpp`）

### include/t1_sim/core/sensor_models.hpp · src/core/sensor_models.cpp
- **职责**：给理想测量值加零偏、噪声、漂移，让补偿和估计在比真值更难的数据上验证。
- **实现要点**：`std::normal_distribution`；`FtSensorModel` 的零偏按 `bias_drift_per_s · sqrt(dt)` 做随机游走（零偏状态存在成员里，不要修改 config）；`ImuModel` 的陀螺零偏在构造时抽一次。
- **测试要点**：同一个 seed 结果可复现；1e4 个样本的均值 ≈ 真值 + 零偏，标准差 ≈ 配置值（±5%）。
- **里程碑** M0（IMU），M3a（力传感器）　**行数预算** ≤ 60

### include/t1_sim/core/sim_viewer.hpp · src/core/sim_viewer.cpp
- **职责**：可选的 GLFW 窗口，在独立线程中以约 60 Hz 渲染，不推进物理。
- **实现要点**
  - 在 `run()` 线程里创建窗口和 OpenGL 上下文（GLFW 要求上下文和使用者在同一线程）。
  - 每帧在锁内 `mjv_updateScene`（或 `mj_copyData` 到私有副本），锁外 `mjr_render`。
  - 鼠标拖动旋转/平移相机，参考 MuJoCo 自带的 `simulate` 示例。
  - `stop()`：`running_ = false` 后 `join`。
- **验收**：仿真启动后看到窗口，关闭窗口不影响控制回路。
- **里程碑** M0　**行数预算** ≤ 150

### include/t1_sim/ros/t1_sim_system.hpp · src/ros/t1_sim_system.cpp
- **职责**：`t1_sim/T1SimSystem` 插件；参数来自 `ros2_control.xacro` 中的 `<param>`。
- **实现要点**
  - `on_init`：
    - 读取 `mjcf_path` 和 `sim_config`（sim.yaml）；
    - 校验 `info.joints` 共 8 个，接口名和顺序与 xacro 一致；
    - 构造 world 和噪声模型，按大小分配所有存储向量。
  - `export_*_interfaces`：按 `info.joints` 的顺序，把存储向量元素的地址交出去。传感器按 `info.sensors` 的名字（`imu`、`ft_sensor`）导出，接口名用标准名字（`orientation.x`、`force.x` 等）。
  - `on_activate`：先读一次状态；把位置指令设为当前位置、刚度设为 0（激活瞬间不跳）。
  - `read`：推进 `n = round(period / timestep)` 步，然后把状态和传感器（加噪声后）写入存储。
  - `write`：把指令存储打包成 `MitCommand` 和 `WheelSpeeds` 交给 world；NaN 当作 0 处理。
  - 在 cpp 末尾 `PLUGINLIB_EXPORT_CLASS`（已写好）。
- **验收**：`ros2 launch t1_bringup sim.launch.py wbc:=false` 后，`ros2 control list_hardware_interfaces` 列出全部接口；`/joint_states` 为 500 Hz。
- **里程碑** M0　**行数预算** ≤ 200

### worlds/flat.xml · worlds/cleaning_wall.xml
- **职责**：场景。flat 为空地；cleaning_wall 有一面竖墙、一个 20° 斜面和带槽位的刷头座。
- **注意**：`urdf_to_mjcf.py --world` 会把机器人合并进去，生成 `t1_description/mjcf/t1_scene.xml`。

### config/sim.yaml
- **职责**：物理步长、是否开窗口、噪声参数、随机种子。

### 计划中
- `core/brush_attachment.hpp/.cpp`（M6）：用 MuJoCo 的 weld equality 模拟磁吸取放刷头。
