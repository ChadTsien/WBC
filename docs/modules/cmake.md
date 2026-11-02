# t1_cmake

> 目录已按 horkin-core 重排成仓库根下的同名目录，不再使用 `src/t1_*` 包。文中若还出现旧路径，以 README 的目录表和 `docs/PROGRESS.md` 为准。


公共 CMake 模块。别的包 `find_package(t1_cmake REQUIRED)` 后即可 `include()` 下面的模块。状态：**完成**（框架阶段由我写好）。

| 文件 | 职责 |
|---|---|
| `t1_cmake-extras.cmake` | 把本包安装的 cmake 目录加进 `CMAKE_MODULE_PATH` |
| `cmake/T1Common.cmake` | C++17、导出 compile_commands、默认 RelWithDebInfo；`t1_target_defaults(目标)` 加统一警告 |
| `cmake/T1Pinocchio.cmake` | 在 `/opt/openrobots` 找 Pinocchio 2.7；找到 3.x 直接报错；结果放在 `${T1_PINOCCHIO}` |
| `cmake/T1ProxSuite.cmake` | 找 ProxSuite，找不到报错并提示安装命令 |
| `cmake/T1MuJoCo.cmake` | 找 `$MUJOCO_DIR`（默认 `~/.mujoco/mujoco-3.9.0`），生成导入目标 `T1::mujoco` |

## 用法

```cmake
find_package(t1_cmake REQUIRED)
include(T1Common)
include(T1Pinocchio)        # 用到 t1_model 或 Pinocchio 的包都要加
add_library(foo SHARED ...)
t1_target_defaults(foo)
```

## 注意

- 下游包只要依赖 `t1_model`（哪怕间接），就要在 `find_package(t1_model)` 之前 `include(T1Pinocchio)`，保证找到的是 2.7。
- `T1ProxSuite` 在实现 `t1_wbc/src/core/qp_solver.cpp` 时启用；`T1MuJoCo` 在实现 `t1_sim/src/core/mujoco_world.cpp` 时启用。
