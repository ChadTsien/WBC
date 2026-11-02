# launch

> 目录已按 horkin-core 重排成仓库根下的同名目录，不再使用 `src/t1_*` 包。文中若还出现旧路径，以 README 的目录表和 `docs/PROGRESS.md` 为准。


启动文件和控制器配置，没有 C++ 代码。

### config/controllers.yaml
- **职责**：controller_manager 的配置：`update_rate: 500`，四个控制器（`joint_state_broadcaster`、`imu_sensor_broadcaster`、`ft_sensor_broadcaster`、`wbc_controller`）及其参数。
- **注意**：`wbc_controller` 的 `arm_joints`、`wheel_joints`、`imu_name`、`ft_name` 必须与 `ros2_control.xacro` 一致。
- **状态**：完成。

### launch/sim.launch.py
- **职责**：robot_state_publisher + ros2_control_node（加载 `T1::sim::SimSystem`）+ 各 spawner + Rerun，可选 Foxglove / RViz。
- **参数**：`wbc`（默认 true；false 时只有广播器）、`rerun`（默认 true）、`rerun_save`（空则只开窗口，填路径则同时写 `.rrd`）、`foxglove`（默认 false）、`rviz`。
- **状态**：完成。M0 阶段仿真插件实现后即可启动。

### tools/rerun_view.py
- **职责**：订阅状态、接触、力、关节、里程计和 tf，记进 Rerun。三维视图看坐标系，时间序列看刷头误差、底盘速度、力和 QP 耗时。
- **不在** `WbcRosController::update` 里调用。控制线程只发布话题。
- **用法**：`ros2 launch wbc sim.launch.py`，或单独 `ros2 run wbc rerun_view.py --save bags/run.rrd`。
- **状态**：完成。

### launch/record.launch.py
- **职责**：用 mcap 格式录包到 `bags/<时间戳>_<name>/`，话题列表写在文件里。
- **用法**：`ros2 launch t1_bringup record.launch.py name=wall_test`；回放后用 Foxglove 打开 `.mcap` 画曲线。
- **状态**：完成。新增话题时记得加进列表。

### 计划中
- `launch/real.launch.py`（M8）：同 sim，换成 `use_sim:=false` 并加载 `motors.yaml`；启动雷达、深度相机驱动。
- `launch/nav.launch.py`（M4）：Nav2 + 定位。
- `config/foxglove_layout.json`（M0 之后随时）：保存常用面板布局：刷头误差、QP 耗时、法向力、底盘速度。
