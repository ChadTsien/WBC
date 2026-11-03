# T1-WBC

移动操作清扫机器人的全身控制。一个 CMake 工程，目录按 horkin-core 的方式切：算法库不含 ROS，接线只在 `ros/`。

C++ 命名空间是 `T1`。文件名、目录名、ROS 包名都不加 `t1`。ROS 包名是 `wbc`。

## 上手

```bash
./scripts/setup_deps.sh                # 不要加 sudo；脚本内部会对 apt 提权
source scripts/env.sh
./scripts/check_env.sh
colcon build --packages-select wbc
ros2 launch wbc sim.launch.py          # 同时打开 Rerun
```

Jetson 上用 `./scripts/setup_deps_jetson.sh`。部署见 `tools/deploy.sh`。

## 目录

| 目录 | 一句话 |
|---|---|
| `types/` | 公共数据类型 |
| `runtime/` | 数学、滤波、实时队列和定时 |
| `model/` `kine/` `dyn/` | Pinocchio 模型、运动学、动力学 |
| `signal/` | 力补偿、标定数据、接触检测 |
| `estimation/` | 轮速里程计、平面 EKF、倾角 |
| `control/` | 全身 QP，不含 ROS |
| `sim/` | MuJoCo 植物和传感器模型 |
| `ros/` | ros2_control 插件和标定节点 |
| `config/` | URDF、网格、YAML、场景 |
| `launch/` | 仿真、录包、查看模型 |
| `tools/rerun_view.py` | 把话题记进 Rerun：三维坐标系和曲线 |
| `msgs/` | 消息和服务定义 |
| `cmake/` | 编译选项、Pinocchio / ProxSuite / MuJoCo |
| `hal/` `planning/` `task/` `perception/` | 后续里程碑再创建，说明在 `docs/modules/` |

头文件从仓库根引用，例如 `#include "kine/arm.hpp"`。

## 文档

| 文档 | 内容 |
|---|---|
| [docs/PLAN.md](docs/PLAN.md) | 总计划 |
| [docs/conventions.md](docs/conventions.md) | 约定 |
| [docs/PROGRESS.md](docs/PROGRESS.md) | 进度 |
| [docs/modules/](docs/modules/) | 每个目录一份实现说明 |
