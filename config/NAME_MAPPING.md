# 原始 URDF 与 wbc 的名称对照

原始文件：`urdf/raw/6-8-1.urdf`（SolidWorks 导出，来自 washcar-simulator/v6_description）。

## 关节

| 原始关节 | 类型 | 新关节 | 说明 |
|---|---|---|---|
| `0` | fixed | `arm_mount_joint` | 车身 → 机械臂底座 |
| `1` | revolute | `arm_joint1` | SCARA 第一关节，绕 z |
| `2` | revolute | `arm_joint2` | SCARA 第二关节，绕 z |
| `3` | prismatic | `arm_joint3` | 丝杠升降，z 向平移 |
| `4` | revolute | `arm_joint4` | 腕部偏航，绕 z |
| `5` | revolute | `arm_joint5` | 俯仰二连杆第一关节 |
| `6` | revolute | `arm_joint6` | 俯仰二连杆第二关节 |
| `tool` | fixed | `tool_joint` | 末端法兰 |
| `zuo` | revolute（限位 0） | `wheel_left_joint` | 改为 continuous |
| `y` | revolute（限位 0） | `wheel_right_joint` | 改为 continuous |
| `zq` `yq` `zh` `yh` | revolute（限位 0） | 删除 | 万向轮改为固定小球 |
| — | — | `chassis_joint` | 新增：base_link → chassis_link |
| — | — | `ft_sensor_joint` `brush_mount_joint` `brush_tip_joint` | 新增末端链 |

## 连杆

| 原始连杆 | 新连杆 |
|---|---|
| — | `base_link`（新增，REP-103） |
| `jisehn` | `chassis_link` |
| `0` | `arm_base_link` |
| `1` … `6` | `arm_link1` … `arm_link6` |
| `tool` | `tool_link` |
| `zuo` / `you` | `wheel_left_link` / `wheel_right_link` |
| `zq` `yq` `zh` `yh` | `caster_front_left` `caster_front_right` `caster_rear_left` `caster_rear_right` |
| — | `ft_sensor` `brush_mount` `brush_tip` `imu_link` `laser_link`（新增） |

## 坐标系差异

- 原始 `jisehn`：+Y 朝前、+Z 朝上、+X 朝右，原点离地约 0.1585 m（轮心 z = -0.093，轮半径 0.0655）。
- 新 `base_link`：+X 朝前、+Z 朝上，原点在两驱动轮轴线中点的地面投影。
- 因此 `chassis_joint` 的旋转为绕 z 轴 -90°。
