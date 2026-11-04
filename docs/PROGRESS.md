# 进度表

**更新方法**：每完成一个文件（实现 + 测试通过），把状态改成"完成"并填上日期；开始写时改成"进行中"。表格按建议的动手顺序排列，前面的被后面的依赖。每个文件具体要做什么、测什么，见 `docs/modules/` 里对应目录的文档。

状态：未开始 / 进行中 / 完成。"框架"表示由框架阶段直接写好，无需手敲。

## M0 建模与仿真

| # | 文件 | 状态 | 日期 |
|---|---|---|---|
| 1 | `types/types.hpp` | 框架 | 2026-11-02 |
| 2 | `runtime/math` | 完成 | 2026-11-03 |
| 3 | `runtime/yaml_params` | 完成 | 2026-11-03 |
| 4 | `runtime/filters` | 完成 | 2026-11-03 |
| 5 | `runtime/rate_limiter` | 完成 | 2026-11-04 |
| 6 | `runtime/spsc_queue.hpp` | 完成 | 2026-11-04 |
| 7 | `config/urdf/chassis.xacro` | 完成 | 2026-11-04 |
| 8 | `config/urdf/arm.xacro` | 完成 | 2026-11-04 |
| 9 | `config/urdf/end_effector.xacro` | 完成 | 2026-11-04 |
| 10 | `tools/urdf_to_mjcf.py` | 未开始 | |
| 11 | `model/robot_model` | 未开始 | |
| 12 | `kine/arm`（位姿、雅可比） | 未开始 | |
| 13 | `kine/base`（互转） | 未开始 | |
| 14 | `sim/world` | 未开始 | |
| 15 | `sim/sensors`（IMU） | 未开始 | |
| 16 | `sim/viewer` | 未开始 | |
| 17 | `ros/sim_system` | 未开始 | |
| — | `tools/rerun_view.py` | 完成 | 2026-11-02 |
| — | M0 验收：Rerun 里能看到坐标系和曲线；Pinocchio 与 MuJoCo 误差 < 0.1 mm | 未开始 | |

## M1 运动学 WBC

| # | 文件 | 状态 | 日期 |
|---|---|---|---|
| 18 | `runtime/rt_thread` | 未开始 | |
| 19 | `runtime/loop_timer` | 未开始 | |
| 20 | `kine/base`（point_jacobian） | 未开始 | |
| 21 | `kine/arm`（manipulability） | 未开始 | |
| 22 | `kine/whole_body` | 未开始 | |
| 23 | `control/qp_problem` | 未开始 | |
| 24 | `control/qp_solver` | 未开始 | |
| 25 | `control/reference` | 未开始 | |
| 26 | `control/task_ee` | 未开始 | |
| 27 | `control/task_posture` | 未开始 | |
| 28 | `control/task_base` | 未开始 | |
| 29 | `control/limits` | 未开始 | |
| 30 | `control/command` | 未开始 | |
| 31 | `control/config` | 未开始 | |
| 32 | `control/controller`（不含接触） | 未开始 | |
| 33 | `ros/io` | 未开始 | |
| 34 | `ros/controller` | 未开始 | |
| — | M1 验收：遥控底盘时刷头误差 < 5 mm | 未开始 | |

## M2 状态估计

| # | 文件 | 状态 | 日期 |
|---|---|---|---|
| 35 | `estimation/odometry` | 未开始 | |
| 36 | `estimation/ekf` | 未开始 | |
| 37 | `estimation/tilt` | 未开始 | |
| — | M2 验收：加噪声和打滑后 M1 指标仍成立 | 未开始 | |

## M3a 力传感器补偿

| # | 文件 | 状态 | 日期 |
|---|---|---|---|
| 38 | `sim/sensors`（力传感器） | 未开始 | |
| 39 | `signal/gravity` | 未开始 | |
| 40 | `signal/identify` | 未开始 | |
| 41 | `signal/inertia` | 未开始 | |
| 42 | `signal/compensate` | 未开始 | |
| 43 | `signal/brushes` | 未开始 | |
| 44 | `dyn/dynamics`（负载） | 未开始 | |
| 45 | `ros/ft_calibration` | 未开始 | |
| — | M3a 验收：空中运动时补偿残余 < 0.5 N | 未开始 | |

## M3b 导纳与接触

| # | 文件 | 状态 | 日期 |
|---|---|---|---|
| 46 | `dyn/dynamics`（重力矩） | 未开始 | |
| 47 | `signal/contact` | 未开始 | |
| 48 | `control/contact_phase` | 未开始 | |
| 49 | `control/admittance` | 未开始 | |
| 50 | `control/controller`（接入接触） | 未开始 | |
| — | M3b 验收：移动中恒力误差 < ±20%；确定清扫力 | 未开始 | |

## M4 以后

文件在里程碑开始时创建，届时把它们加到这里。计划见 `docs/modules/` 里的 `planning`、`task`、`perception`、`hal`，以及 `model`、`control`、`sim` 文档末尾的"计划中"。

## 环境

| 事项 | 状态 | 日期 |
|---|---|---|
| `./scripts/setup_deps.sh`，`check_env.sh` 全过 | 未开始 | |
| `git init` 并首次提交 | 未开始 | |
| Jetson：`setup_deps_jetson.sh` | 未开始 | |
| 交叉编译：`sync_sysroot.sh` + `deploy.sh` 跑通 | 未开始 | |
