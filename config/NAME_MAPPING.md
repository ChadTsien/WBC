# 原始 URDF 与 wbc 的名称对照

当前 CAD：`urdf/raw/v6_wbc.urdf`（SolidWorks，2026-11-05，机器人名 `11-4-2`）。零位 rpy 已清装配残差；J4 用 `π/2`（CAD 写 1.5807）。
旧导出 `urdf/raw/6-8-1.urdf` 只留档，xacro 不再引用。

SolidWorks 把关节和子连杆起同一个名字。xacro 里拆开：关节 `*_joint`，连杆 `*_link`。

## 关节

| CAD 关节 | 类型 | xacro 关节 | 说明 |
|---|---|---|---|
| `wheel_left_link` | continuous | `wheel_left_joint` | 左驱动轮 |
| `wheel_right_link` | continuous | `wheel_right_joint` | 右驱动轮 |
| `arm_mount_joint` | fixed | `arm_mount_joint` | 车身 → 机械臂底座 |
| `arm_joint1` | revolute | `arm_joint1` | SCARA 第一关节，绕 z |
| `arm_joint2` | revolute | `arm_joint2` | SCARA 第二关节，绕 z |
| `arm_joint3` | prismatic | `arm_joint3` | 丝杠升降 |
| `arm_joint4` | revolute | `arm_joint4` | 腕部偏航，绕 z |
| `arm_joint5` | revolute | `arm_joint5` | 俯仰二连杆第一关节 |
| `arm_joint6` | revolute | `arm_joint6` | 俯仰二连杆第二关节 |
| `tool_joint` | fixed | `tool_joint` | 末端法兰 |
| — | — | `chassis_joint` | 新增：base_link → chassis_link |
| — | — | `ft_sensor_joint` `brush_mount_joint` `brush_tip_joint` | 新增末端链 |
| — | — | 万向轮关节 | CAD 无万向轮；xacro 用固定小球 |
| — | revolute | `chassis_rgbd_joint` | 机身 RGBD 偏航；不进 WBC |
| — | fixed | `arm_rgbd_joint` | RGBD 固连 `arm_link5` |

## 连杆

| CAD 连杆 | xacro 连杆 | 网格 |
|---|---|---|
| — | `base_link`（新增，REP-103） | 无 |
| `chassis_link` | `chassis_link` | `chassis_link.STL` |
| `wheel_left_link` / `wheel_right_link` | 同名 | 同名 STL |
| `arm_mount_joint` | `arm_base_link` | `arm_mount_joint.STL` |
| `arm_joint1` … `arm_joint6` | `arm_link1` … `arm_link6` | `arm_jointN.STL` |
| `tool_joint` | `tool_link` | `tool_joint.STL` |
| — | `caster_*` `imu_link` `laser_link` `ft_sensor` `brush_mount` `brush_tip` | 无 / 简单几何 |
| — | `chassis_rgbd_link` `chassis_rgbd_optical` `arm_rgbd_link` `arm_rgbd_optical` | 小盒子 / 光学系 |

## 坐标系

- `v6_wbc` 的 `chassis_link` 已经是 +X 朝前、+Y 朝左、+Z 朝上。驱动轮在 `x=0.16`、`y=±0.20275`、`z=0.0237`。
- `base_link`：+X 朝前，原点在两驱动轮轴线中点的地面投影。
- `chassis_joint` 只平移、不旋转：`xyz = (−axle_x, 0, wheel_radius − axle_z)`。
