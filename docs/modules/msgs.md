# t1_msgs

> 目录已按 horkin-core 重排成仓库根下的同名目录，不再使用 `src/t1_*` 包。文中若还出现旧路径，以 README 的目录表和 `docs/PROGRESS.md` 为准。


层与层之间的消息和服务。状态：**完成**（接口定义由我写好，改动需同步改所有使用方）。

## 消息

| 文件 | 用途 | 发布者 → 订阅者 |
|---|---|---|
| `msg/TaskSpacePoint.msg` | 参考轨迹上的一个点：底盘模式与参考、刷头位姿与速度、接触阶段、表面法向、期望法向力 | — |
| `msg/TaskSpaceTrajectory.msg` | 带起始时间的点序列，`frame_id` 为 odom | 规划 / 任务 / 标定节点 → `wbc_controller/reference` |
| `msg/WbcStatus.msg` | QP 状态、迭代次数、耗时、末端误差、底盘与关节速度指令、导纳修正量、可操作度 | `wbc_controller/status` → 日志、任务层 |
| `msg/ContactState.msg` | 接触阶段、是否接触、是否脱落、法向力、世界系外力 | `wbc_controller/contact` → 任务层 |
| `msg/FtStatus.msg` | 力传感器标定状态、当前刷头、质量、质心、零偏 | `wbc_controller/ft_status` → 日志 |

## 服务

| 文件 | 服务端 | 用途 |
|---|---|---|
| `srv/TareFt.srv` | `wbc_controller/tare_ft` | 重新标零（调用方保证刷头悬空且静止） |
| `srv/SetBrush.srv` | `wbc_controller/set_brush` | 切换刷头，加载其重力参数 |
| `srv/CalibrateFt.srv` | `ft_calibration/calibrate` | 执行完整重力辨识并保存 |

## 约定

- 所有位姿、速度、力都在 `TaskSpaceTrajectory.header.frame_id`（odom）下表达。
- `ee_pose` 的 z 轴是刷头轴线，指向刷面外侧；绕 z 的旋转会被忽略。
- `surface_normal` 指向表面外侧；`desired_normal_force` 为正表示压向表面。
