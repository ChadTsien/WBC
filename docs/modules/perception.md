# t1_perception（M7，目前只有包壳）

> 目录已按 horkin-core 重排成仓库根下的同名目录，不再使用 `src/t1_*` 包。文中若还出现旧路径，以 README 的目录表和 `docs/PROGRESS.md` 为准。


深度相机识别脏区，输出要清扫的区域（多边形 + 平面），交给 `t1_planning` 生成覆盖路径。命名空间 `T1::perc`。

## 处理流程
1. 彩色图 → 脏污分割（先用颜色/纹理阈值做基线，再换成轻量分割网络，用 TensorRT 在 Orin Nano 上跑）。
2. 掩码 + 深度 → 点云 → RANSAC 拟合平面，得到表面法向。
3. 掩码轮廓投影到平面 → 简化的多边形，膨胀一个刷头半径。
4. 转到 odom 系发布。

## 计划文件（未创建）

| 文件 | 职责 |
|---|---|
| `core/dirt_segmenter.hpp/.cpp` | 分割接口；基线实现为阈值法 |
| `core/surface_fitter.hpp/.cpp` | 点云平面拟合、法向朝向相机一侧 |
| `core/region_extractor.hpp/.cpp` | 掩码 → 多边形（OpenCV `findContours` + `approxPolyDP`） |
| `ros/perception_node.cpp` | 同步彩色与深度图像，发布区域消息 |
| `msg`（加到 t1_msgs） | `CleanRegion`：polygon、plane normal、置信度 |

## 测试要点
- 平面拟合：合成带噪声的平面点云，法向误差 < 2°。
- 区域提取：合成矩形掩码，多边形面积误差 < 5%。
- 仿真：在 MuJoCo 墙面贴脏污纹理，渲染深度相机图像做端到端验证。
