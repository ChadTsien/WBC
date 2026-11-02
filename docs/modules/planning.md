# t1_planning（M4–M5，目前只有包壳）

> 目录已按 horkin-core 重排成仓库根下的同名目录，不再使用 `src/t1_*` 包。文中若还出现旧路径，以 README 的目录表和 `docs/PROGRESS.md` 为准。


生成 WBC 要跟踪的 `TaskSpaceTrajectory`：底盘的全局路径（Nav2）、刷头的覆盖轨迹、取放刷头等动作。它只输出参考，不做实时控制；实时避障在 WBC 的 QP 约束里。命名空间 `T1::plan`。

## 计划文件（未创建）

| 文件 | 职责 | 要点 |
|---|---|---|
| `core/coverage_planner.hpp/.cpp` | 平面多边形区域 → 往返（弓字形）刷头路径 | 行距 = 刷头直径 × (1 − 重叠率)；区域先投影到表面平面；转弯处插圆弧 |
| `core/time_parameterization.hpp/.cpp` | 几何路径 → 带时间的轨迹 | 梯形速度（或 TOPP-RA）；接触段恒速；输出总时长，供任务层倒排 |
| `core/motion_library.hpp/.cpp` | 预定义动作：离开收纳位、取刷头、放刷头、收回 | 以关节或刷头关键帧描述，存 yaml；时长固定、已知 |
| `core/base_goal_selector.hpp/.cpp` | 给定清扫区域，选择底盘停靠位姿 | 在候选位姿上检查机械臂可达性和可操作度，选最好的；够不着时把区域切分成多个停靠点 |
| `ros/planning_node.hpp/.cpp` | 动作服务器：PlanCoverage、ExecuteMotion | 调用 Nav2 的 NavigateToPose；发布参考到 `wbc_controller/reference` |
| `config/nav2_params.yaml` | Nav2 参数 | 差速底盘用 RPP 或 MPPI；footprint 包含收拢状态的机械臂 |

## 设计约定
- 所有轨迹都在 odom 系下发布。由 map 系转换时使用当时的 `map → odom`，发出后不再修正。
- 清扫时底盘模式为 FREE（让 WBC 自己分配）；导航时为 TRACK_TWIST（由 Nav2 控制器给速度）。
