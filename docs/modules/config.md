# t1_description

> 目录已按 horkin-core 重排成仓库根下的同名目录，不再使用 `src/t1_*` 包。文中若还出现旧路径，以 README 的目录表和 `docs/PROGRESS.md` 为准。


机器人模型的唯一来源。原始导出文件只读留档，所有整理在 xacro 里完成；MJCF 由脚本从 URDF 生成，两边永远一致。名称对照见 `config/NAME_MAPPING.md`。

## 整理清单（M0，全部在 xacro / yaml 中完成）

| # | 问题 | 改哪里 |
|---|---|---|
| 1 | 关节 effort/velocity 为 0，J1/J2/J4 位置限位是 ±3.14 占位值 | `config/joint_limits.yaml`（先用估计值），`arm.xacro` 从 yaml 读取 |
| 2 | 驱动轮是被限位 0 焊死的 revolute | `chassis.xacro`：改为 `continuous` |
| 3 | 万向轮只有一个水平轴，关节原点不在轮心 | `chassis.xacro`：删关节，改为固定小球连杆，位置读 `chassis.yaml` |
| 4 | 轮距 0.406（URDF）与 0.402（旧仿真）不一致 | `chassis.yaml`，实测后修改 |
| 5 | 车头朝 +Y，根坐标系离地约 0.16 m | 已由 `v6_wbc.urdf` 解决：`chassis_link` 为 +X 朝前；`chassis_joint` 只平移 |
| 6 | 关节/连杆名为数字，网格路径混用两个包名 | 所有 xacro 用新名字；网格统一为 `package://wbc/meshes/raw/` |
| 7 | 缺少力传感器、刷头 | `end_effector.xacro`：`ft_sensor`、`brush_mount`、`brush_tip` |

另外两件事：碰撞体用简化几何体（放在 `meshes/collision/`，或直接用 box/cylinder/capsule），显示仍用原始网格；车身质量 50 kg 是整数，待称重。

---

### urdf/raw/v6_wbc.urdf · meshes/raw/*.STL
- **职责**：当前 SolidWorks 导出，**只读**。旧档 `6-8-1.urdf` 仍留在 `urdf/raw/`。拿到新 CAD 时替换 `v6_wbc.urdf` 和对应 STL，再检查 xacro。

### urdf/t1.urdf.xacro
- **职责**：主文件。读入三个 yaml，组装 chassis / arm / end_effector，按 `use_ros2_control` 决定是否加 `<ros2_control>`。
- **参数**：`use_sim`（true：`t1_sim/T1SimSystem`；false：`t1_hardware/T1HardwareSystem`）、`use_ros2_control`。
- **状态**：完成。

### urdf/cameras.xacro · config/cameras.yaml
- **职责**：机身可偏航 RGBD（`chassis_rgbd_*`）、J5 上固定 RGBD（`arm_rgbd_*`），各带光学系。不进 `ros2_control` / WBC。
- **里程碑** M0（占位）；安装位实测后改 yaml。

### urdf/chassis.xacro
- **职责**：`base_link`、`chassis_joint`、`chassis_link`、两个驱动轮、四个万向轮小球、`imu_link`、`laser_link`。
- **实现要点**
  - `chassis_joint`：CAD 已是 +X 朝前，`rpy="0 0 0"`；平移 `xyz=(-axle_x, 0, wheel_radius − axle_z)`，使 `base_link` 原点在驱动轴中点地面投影。
  - 驱动轮：`continuous`；CAD 的 child +Z 经 rpy 后沿底盘 −Y，`axis="0 0 -1"` 使正转 = 车往前。
  - 惯量照抄 `v6_wbc.urdf` 的 `chassis_link` / `wheel_*_link`。
- **验收**：`ros2 launch wbc view.launch.py` 中，车头沿 RViz 的红轴（+X），轮子贴地（z = 0）。
- **里程碑** M0

### urdf/arm.xacro
- **职责**：`arm_mount_joint`、`arm_joint1..6`、`arm_link1..6`、`tool_joint`。
- **实现要点**：`origin`、`axis`、`inertial` 逐个照抄原始 URDF（对照 NAME_MAPPING）；`limit` 从 `limits[name]` 读取；J3 为 `prismatic`。
- **验收**：用 joint_state_publisher_gui 逐个拖动关节，运动方向与实物一致（J1/J2/J4 绕竖直轴，J3 上下，J5/J6 俯仰）。
- **里程碑** M0

### urdf/end_effector.xacro
- **职责**：`tool_link → ft_sensor → brush_mount → brush_tip`，全部 fixed。
- **实现要点**：`ft_sensor` 是测量坐标系（按传感器手册定义原点和轴向）；`brush_tip` 在刷面中心，z 轴指向刷面外侧；刷头的质量和惯量挂在 `brush_mount` 上（用圆柱公式估算）。
- **里程碑** M0

### urdf/ros2_control.xacro
- **职责**：仿真、实机、WBC 三方共用的接口契约。
- **状态**：完成。改动必须同时改 `T1SimSystem`、`T1HardwareSystem`、`WbcRosController`。

### config/joint_limits.yaml · config/chassis.yaml · config/end_effector.yaml
- **职责**：模型参数。标注"est"的都是估计值，实测后直接改这里。

### scripts/urdf_to_mjcf.py
- **职责**：xacro → URDF → MJCF，并合并进一个 world 文件。
- **实现要点**
  - `run_xacro`：`subprocess.run(["xacro", path, "use_ros2_control:=false"])`。
  - `urdf_to_mjcf`：URDF 里的 `package://` 路径换成绝对路径（或在 `<mujoco><compiler meshdir=.../>` 中指定）；`mujoco.MjModel.from_xml_string` 加载后用 `mujoco.mj_saveLastXML` 导出。
  - `add_actuators_and_sensors`（用 `xml.etree` 修改）：
    - 底盘加 `freejoint`；
    - 每个机械臂关节一个 `motor` 执行器（力矩由硬件插件按 MIT 公式算）；
    - 驱动轮用 `velocity` 执行器；
    - 万向轮小球摩擦设得很低；
    - 传感器：IMU 用 `framequat` + `gyro` + `accelerometer`（site 放在 `imu_link`），力传感器用 `force` + `torque`（site 放在 `ft_sensor`）。
  - `wrap_in_world`：把 world 文件里的 `worldbody` 子元素、`asset`、`option` 合并进来。
- **验收**：生成的 `mjcf/t1_scene.xml` 能用 `python3 -m mujoco.viewer --mjcf=...` 打开，机器人站在地上不抖；Pinocchio 与 MuJoCo 在随机关节角下 `brush_tip` 位置误差 < 0.1 mm。
- **里程碑** M0　**行数预算** ≤ 200

### launch/view.launch.py
- **职责**：RViz 查看模型 + 关节滑条。**状态**：完成。
