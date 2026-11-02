# t1_hardware（M8，目前只有包壳）

> 目录已按 horkin-core 重排成仓库根下的同名目录，不再使用 `src/t1_*` 包。文中若还出现旧路径，以 README 的目录表和 `docs/PROGRESS.md` 为准。


实机的 ros2_control 硬件插件 `t1_hardware/T1HardwareSystem`。导出的接口与 `T1SimSystem` 完全一致（契约见 `t1_description/urdf/ros2_control.xacro`），控制器无需任何改动。命名空间 `T1::hw`。

## 关键约束

- **CAN 带宽**：MIT 模式每个电机每周期一收一发约 260 bit，6 个电机约 1560 bit；在 1 Mbps、500 Hz 下占用约 78%，余量太小。建议用两路 USB-CAN（每路 3 个电机），或把回路降到 400 Hz。
- **USB 延迟**：达妙官方 USB-CAN 是 USB 转串口协议，不走 SocketCAN。要实测往返延迟的分布，P99 必须小于 1 ms。
- **实时线程**：CAN 收发放在独立的 RT 线程（`apply_rt_config` 绑到一个隔离的核），与 controller_manager 线程之间用无锁双缓冲交换数据。

## 计划文件（未创建）

| 文件 | 职责 | 要点 |
|---|---|---|
| `core/damiao_protocol.hpp/.cpp` | MIT 帧的打包与解析 | 浮点 ↔ 定点：`uint = (x − min)/(max − min)·(2^bits − 1)`；q 16 位、dq 12 位、kp 12 位、kd 12 位、τ 12 位；各电机型号的 PMAX/VMAX/TMAX 放 yaml |
| `core/usb_can.hpp/.cpp` | 达妙 USB-CAN 的串口收发 | 按官方协议文档封帧；非阻塞读；断线检测 |
| `core/motor_bus.hpp/.cpp` | 一路 CAN 上的若干电机：使能、失能、清错、周期收发 | 收发线程、超时计数、电机错误码 |
| `core/wheel_driver.hpp/.cpp` | 底盘轮电机驱动 | 取决于轮电机型号 |
| `ros/t1_hardware_system.hpp/.cpp` | 插件本体 | 结构对照 `T1SimSystem`；`on_activate` 先读位置再使能，第一帧发当前位置、kp = 0 |
| `config/motors.yaml` | 电机 ID、型号、方向、零位偏移、所在 CAN 通道 | 方向和零位用来对齐 URDF 定义 |

## 测试要点（届时细化）
- 协议：打包后再解析误差 < 1 个量化步长；边界值饱和而不是溢出。
- 台架：单电机 MIT 阶跃；6 电机 500 Hz 连续运行 10 min，统计丢帧和延迟。
- 安全：拔掉 USB 后 50 ms 内所有电机失能（电机侧也要设置通信超时保护）。
