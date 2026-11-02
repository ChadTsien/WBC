# t1_common

> 目录已按 horkin-core 重排成仓库根下的同名目录，不再使用 `src/t1_*` 包。文中若还出现旧路径，以 README 的目录表和 `docs/PROGRESS.md` 为准。


不依赖 ROS 的基础库，所有包都会用到。全部是 core，命名空间 `T1`。**建议作为第一个动手的包**，用来熟悉"头文件已给定 → 写实现 → 写测试 → 打勾"的流程。

---

### include/t1_common/types.hpp
- **职责**：全项目共用的类型和维度常量：`Vec3/Vec5/ArmVec/WbcVec`、`Wrench`、`Twist`、`Pose2d`、`BaseTwist`、`ArmState`、`MitCommand`。
- **状态**：完成（纯声明）。
- **注意**：`kNumWbcDof = 8`（v、ω + 6 个关节），`kTaskDim = 5`（位置 3 + 刷头轴线指向 2）。改这里会牵动 QP 维度。

### include/t1_common/math_utils.hpp · src/math_utils.cpp
- **职责**：小工具函数 `wrap_angle`、`skew`、`so3_log`、`yaw_of`、`saturate_norm`。
- **实现要点**
  - `wrap_angle`：`std::atan2(std::sin(a), std::cos(a))`，结果在 (−π, π]；注意 −π 要映射到 π。
  - `skew(v)`：反对称矩阵，满足 `skew(a) * b == a.cross(b)`。
  - `so3_log`：用 `Eigen::AngleAxisd(R)` 得到轴角，返回 `angle * axis`；角度接近 0 时返回零向量。
  - `yaw_of(R)`：`atan2(R(1,0), R(0,0))`。
  - `saturate_norm`：范数超过上限时等比缩放。
- **测试要点**
  - `wrap_angle(3π/2) ≈ −π/2`，`wrap_angle(−π) == π`，`wrap_angle(7π) ≈ π`。
  - 随机 a、b：`skew(a)*b` 与 `a.cross(b)` 误差 < 1e-12。
  - `so3_log(AngleAxis(0.3, n))` ≈ `0.3 n`；单位阵 → 零向量；角度为 π 时轴向正确（符号可以相反）。
  - `saturate_norm`：未超限时原样返回，超限时方向不变、范数等于上限。
- **里程碑** M0　**行数预算** ≤ 60

### include/t1_common/filters.hpp · src/filters.cpp
- **职责**：`LowPass1`（一阶低通）、`Butterworth2`（二阶巴特沃斯低通）、`FilterBank<Filter, N>`（向量版本，已写好）。
- **实现要点**
  - `LowPass1`：`RC = 1/(2π fc)`，`dt = 1/fs`，`alpha = dt/(RC + dt)`；第一次 `update` 直接用输入初始化；之后 `y += alpha (x − y)`。
  - `Butterworth2`（双线性变换）：`K = tan(π fc / fs)`，`norm = 1/(1 + √2 K + K²)`，`b0 = K² norm`，`b1 = 2 b0`，`b2 = b0`，`a1 = 2(K² − 1) norm`，`a2 = (1 − √2 K + K²) norm`；`y = b0 x + b1 x1 + b2 x2 − a1 y1 − a2 y2`。
  - `reset(v)`：把所有历史状态设为 v（稳态），避免启动瞬态。
- **测试要点**
  - 两种滤波器的直流增益为 1：输入恒为 1，足够多步后输出 ≈ 1。
  - `Butterworth2`：fs = 500、fc = 30 时输入 30 Hz 正弦，稳态幅值 ≈ 0.707（±3%）；输入 150 Hz 正弦，幅值 < 0.1。
  - `reset(5)` 后输入 5，输出立即为 5。
- **里程碑** M0　**行数预算** ≤ 80

### include/t1_common/rate_limiter.hpp · src/rate_limiter.cpp
- **职责**：按最大变化率逼近目标值。导纳和接触阶段的期望力斜坡都用它。
- **实现要点**：`step = clamp(target − value, −rate·dt, +rate·dt)`；`rate ≤ 0` 视为不限速。
- **测试要点**：rate = 10、dt = 0.01，目标从 0 跳到 1，10 步后正好到 1；目标下降时对称；不会越过目标。
- **里程碑** M0　**行数预算** ≤ 30

### include/t1_common/spsc_queue.hpp
- **职责**：无锁单生产者单消费者队列（模板，实现写在头文件里）。实时线程把日志数据交给普通线程时用。
- **实现要点**
  - `tail_` 只由生产者写，`head_` 只由消费者写；下标对 `Capacity` 取模。
  - 满的条件：`(tail + 1) % Capacity == head`，因此最多存 `Capacity − 1` 个元素。
  - 内存序：生产者写数据后 `tail_.store(..., release)`；消费者 `tail_.load(acquire)` 后读数据。`head_` 对称处理。
- **测试要点**
  - 单线程：按顺序 push/pop，顺序一致；满了以后 push 返回 false；空了 pop 返回 false。
  - 双线程：生产者 push 1e6 个递增整数（满了就重试），消费者收到的序列严格递增且不丢失。
- **里程碑** M0　**行数预算** ≤ 70

### include/t1_common/rt_thread.hpp · src/rt_thread.cpp
- **职责**：把当前线程设为实时线程：绑核、SCHED_FIFO、锁内存。
- **实现要点**：`pthread_setaffinity_np`、`pthread_setschedparam(SCHED_FIFO)`、`mlockall(MCL_CURRENT | MCL_FUTURE)`；失败时在 `error` 中写明原因，并提示检查 `/etc/security/limits.conf` 的 `rtprio` 和 `memlock`。
- **测试要点**：`cpu = 0, fifo_priority = 0` 应成功；在没有权限的环境里设置优先级 80 应返回 false 且 `error` 非空（不要求测试必须有 root）。
- **里程碑** M1　**行数预算** ≤ 50

### include/t1_common/loop_timer.hpp · src/loop_timer.cpp
- **职责**：`PeriodicTimer`（绝对时刻周期睡眠，不漂移）和 `LoopStats`（耗时统计）。
- **实现要点**
  - `PeriodicTimer`：`clock_gettime(CLOCK_MONOTONIC)` 记起点；`wait()` 时 `next += period`，再 `clock_nanosleep(..., TIMER_ABSTIME, &next)`；若醒来前已超时，`overruns_++`，并把 `next` 跳到当前时刻之后的下一个周期点（丢弃错过的周期，不追赶）。
  - `LoopStats`：环形缓冲写样本（不分配内存）；`percentile` 拷贝到临时 vector 后用 `std::nth_element`（只在非实时侧调用）。
- **测试要点**
  - 1 kHz 跑 1 秒：总耗时 ≈ 1 s（±2%），`overruns` 很少。
  - `LoopStats` 填入 1..100：P50 ≈ 50，P99 ≈ 99，`max` = 100；窗口满后旧样本被覆盖。
- **里程碑** M1　**行数预算** ≤ 90

### include/t1_common/yaml_params.hpp · src/yaml_params.cpp
- **职责**：读取 YAML 参数：`load_yaml_file`、`require<T>`、`get_or<T>`（模板已写好）、`require_vector`。
- **实现要点**：`YAML::LoadFile` 包一层 try/catch，重新抛出的异常信息里带上文件路径；`require_vector` 检查节点是序列，按 `expected_size` 校验长度。
- **测试要点**：用 `/tmp` 下的临时文件测试：正常读取；缺少键时抛出且信息包含键名；向量长度不对时抛出；文件不存在时抛出且信息包含路径。
- **里程碑** M0　**行数预算** ≤ 50
