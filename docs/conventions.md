# 工程约定

这些是我们一起遵守的约定，不做脚本强制检查。代码评审时互相提醒。

## 1. 代码规模

| 对象 | 上限 |
|---|---|
| 单个 `.cpp` / `.hpp` 文件 | 300 行（目标 200 行以内） |
| 单个 Python / launch 文件 | 200 行 |
| 单个函数 | 50 行 |
| 嵌套层数 | 3 层 |
| 单行宽度 | 100 字符（`.clang-format` 已设置） |

- 只算有效代码，空行和注释不计。URDF、MJCF、YAML、自动生成的代码不受限。
- 快超出时优先拆分：一个文件只放一个主要的类；一个函数只做一件事。
- 每个文件在 `docs/modules/*.md` 里有"行数预算"，实现时作为参考。

## 2. 层级结构

一个 CMake 工程，目录和 horkin-core 一样：头文件和源文件放在同一目录，从仓库根引用。

```cpp
#include "types/types.hpp"
#include "kine/arm.hpp"
#include "control/controller.hpp"
```

| 目录 | 职责 | 允许依赖 |
|---|---|---|
| `types/` | 公共数据类型 | 无 |
| `runtime/` | 数学、滤波、实时工具 | `types` |
| `model/` `kine/` `dyn/` | 刚体模型、运动学、动力学 | 往下 |
| `signal/` | 力补偿、接触 | `runtime` |
| `estimation/` | 里程计、EKF | `kine` |
| `control/` | 全身 QP | kine、dyn、signal、estimation |
| `sim/` | MuJoCo，不含 ROS | `kine` |
| `hal/` | 真机驱动，M8 再创建 | 端口和协议 |
| `ros/` | 唯一允许出现 ROS 头文件的目录 | 上面全部 |
| `config/` `launch/` `msgs/` | 模型、参数、消息 | — |

依赖只许向下。`control`、`kine`、`signal` 里不出现 rclcpp、MuJoCo、CAN。仿真插件和控制器插件都在 `ros/`。

ROS 包名是 `wbc`（没有 `t1` 前缀）。消息类型在全局命名空间 `::wbc::msg`，因为代码里已经有 `namespace T1::wbc`，引用消息时要写 `::wbc::msg::WbcStatus`。

编译选项在 `cmake/Defaults.cmake` 的 `target_defaults()`。新目录加一个 `CMakeLists.txt`，再在根 `CMakeLists.txt` 里 `add_subdirectory`。

## 3. 命名与风格

- C++17。`.clang-format`：4 空格，大括号另起一行，命名空间内容缩进，行宽 100。改完用编辑器格式化一次。
- 文件名、目录名不加 `t1`。命名空间写成 `T1`。
- 类名 `PascalCase`；函数和变量 `snake_case`；私有成员加 `_` 后缀；常量 `kPascalCase`。
- 命名空间：`T1`，子模块 `T1::wbc`、`T1::ft`、`T1::est`、`T1::sim`、`T1::hal`。
- 不写魔法数字，参数一律进 YAML。
- 单位：国际单位制，角度用弧度。变量名或注释里写清楚单位和坐标系，例如 `gravity_in_ft`、`tool_in_base`。

## 4. 注释与文档

- 注释用中文，写代码本身看不出来的东西：成员、单位、坐标系、前置条件、流程。风格对齐 horkin-core 的块注释。
- 不写"下一行做什么"。
- `TODO(Mx)` 标记还没实现的地方，`x` 是里程碑编号。
- 设计和实现说明写在 `docs/modules/`，改了接口就同步更新文档。

## 5. 实时规则

在 `update()` 及它调用的所有函数（控制器、硬件插件的 read/write）中：

- 不分配内存：不用 `new`、`push_back`、`std::string` 拼接，不把固定大小矩阵传给 `const MatX&`（用 `Eigen::Ref`）。
- 不加阻塞锁：线程间用 `realtime_tools::RealtimeBuffer`、`RealtimePublisher`、`std::atomic`、`T1::SpscQueue`。
- 不打印日志：用 `RCLCPP_*_THROTTLE`，或把数据放进状态消息。
- 不做文件 I/O。
- 所有内存在构造或 `on_configure` 中一次性分配。

## 6. 测试

- 每个 core 文件都有对应的 gtest，测试由你手写，测试要点写在 `docs/modules/<包名>.md`。
- 测试文件命名 `test/test_<文件名>.cpp`，在 `CMakeLists.txt` 的 `if(BUILD_TESTING)` 块中注册：

  ```cmake
  ament_add_gtest(test_filters runtime/test/test_filters.cpp)
  target_link_libraries(test_filters runtime)
  ```

- 运行：`colcon test --packages-select <包名> && colcon test-result --verbose`
- 一个文件"完成"的标准：测试要点全部覆盖且通过，然后在 `PROGRESS.md` 里打勾。

## 7. 常用命令

```bash
source scripts/env.sh
colcon build --packages-select wbc
colcon build --packages-select wbc --cmake-args -DCMAKE_BUILD_TYPE=Debug
colcon test --packages-select wbc && colcon test-result --verbose
ros2 launch wbc sim.launch.py
ros2 launch wbc record.launch.py name=wall_test
ros2 launch wbc view.launch.py
```
