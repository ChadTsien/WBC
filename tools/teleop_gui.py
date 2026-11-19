#!/usr/bin/env python3
"""遥控与模式试验 GUI。

底盘 cmd_vel 带加速度斜坡，避免阶跃把车掀翻。末端按钮发 /wbc_controller/reference，
用来测 kWholeBody / kArmRelative / kTeleopRelative。

    ros2 run wbc teleop_gui.py
    ros2 launch wbc sim.launch.py
"""
import math
import sys
import threading

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError as exc:
    raise SystemExit("需要 python3-tk：sudo apt install python3-tk") from exc

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Pose, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from wbc.msg import TaskSpacePoint, TaskSpaceTrajectory, WbcStatus

PUB_HZ = 20.0
DT = 1.0 / PUB_HZ
QP_NAME = {0: "OK", 1: "不可行", 2: "超迭代", 3: "未跑"}
FRAME_NAME = {
    TaskSpacePoint.EE_ODOM: "odom 全身",
    TaskSpacePoint.EE_BASE: "base_link 相对",
    TaskSpacePoint.EE_MOUNT: "臂座相对",
}
ARM_JOINTS = (
    "arm_joint1", "arm_joint2", "arm_joint3",
    "arm_joint4", "arm_joint5", "arm_joint6",
)
PRISMATIC = {"arm_joint3"}


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def slew(cur, tgt, max_step):
    dv = tgt - cur
    if abs(dv) <= max_step:
        return tgt
    return cur + math.copysign(max_step, dv)


def to_dur(t):
    d = DurationMsg()
    sec = int(t)
    d.sec = sec
    d.nanosec = int(round((t - sec) * 1e9))
    return d


def tf_to_pose(tf: TransformStamped) -> Pose:
    p = Pose()
    p.position.x = tf.transform.translation.x
    p.position.y = tf.transform.translation.y
    p.position.z = tf.transform.translation.z
    p.orientation = tf.transform.rotation
    return p


def quat_rpy(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(clamp(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def fmt_pose(prefix, pose: Pose):
    p = pose.position
    r, pit, y = quat_rpy(pose.orientation)
    return (
        f"{prefix}  x={p.x:+.3f} y={p.y:+.3f} z={p.z:+.3f} m   "
        f"rpy=[{math.degrees(r):+.1f}, {math.degrees(pit):+.1f}, {math.degrees(y):+.1f}]°"
    )


def shifted(pose: Pose, dx, dy, dz) -> Pose:
    out = Pose()
    out.orientation = pose.orientation
    out.position.x = pose.position.x + dx
    out.position.y = pose.position.y + dy
    out.position.z = pose.position.z + dz
    return out


class TeleopGui(Node):
    def __init__(self):
        super().__init__("teleop_gui")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._cmd_pub = self.create_publisher(Twist, "/wbc_controller/cmd_vel", 10)
        self._traj_pub = self.create_publisher(TaskSpaceTrajectory, "/wbc_controller/reference", 10)
        self.create_subscription(WbcStatus, "/wbc_controller/status", self._on_status, qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos)
        self.create_subscription(JointState, "/joint_states", self._on_joints, qos)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self.create_timer(DT, self._tick)

        self._v_max = 0.12
        self._w_max = 0.35
        self._a_max = 0.20
        self._alpha_max = 0.50
        self._keys = set()
        self._stick = (0.0, 0.0)
        self._latch = False
        self._latched = (0.0, 0.0)
        self._v_cmd = 0.0
        self._w_cmd = 0.0
        self._sending = False
        self._status = "等待 /wbc_controller/status"
        self._odom = "等待 /odom"
        self._obs_chassis = "底盘速度：等待 /odom"
        self._obs_arm = "臂关节：等待 /joint_states"
        self._obs_ee_odom = "末端 odom：等待 tf"
        self._obs_ee_mount = "末端 臂座：等待 tf"
        self._hint = "先点窗口。W/S 前后，A/D 转向，Q/E 原地转。松开会斜坡减速。"
        self._traj_note = "无轨迹（Hold）"
        self._traj_frame = None

        self._root = tk.Tk()
        self._root.title("T1 遥控与模式")
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._build()
        self._root.bind("<KeyPress>", self._on_key)
        self._root.bind("<KeyRelease>", self._on_key)
        self._root.after(50, self._refresh)
        self._root.focus_force()

    def _build(self):
        pad = {"padx": 8, "pady": 4}
        root = ttk.Frame(self._root, padding=10)
        root.grid()

        self._status_var = tk.StringVar(value=self._status)
        self._odom_var = tk.StringVar(value=self._odom)
        self._cmd_var = tk.StringVar(value="底盘：未发送")
        self._traj_var = tk.StringVar(value=self._traj_note)
        self._hint_var = tk.StringVar(value=self._hint)
        ttk.Label(root, textvariable=self._status_var, font=("monospace", 10)).grid(
            row=0, column=0, columnspan=2, sticky="w", **pad)
        ttk.Label(root, textvariable=self._odom_var, font=("monospace", 10)).grid(
            row=1, column=0, columnspan=2, sticky="w", **pad)
        ttk.Label(root, textvariable=self._cmd_var, font=("monospace", 10)).grid(
            row=2, column=0, columnspan=2, sticky="w", **pad)
        ttk.Label(root, textvariable=self._traj_var, font=("monospace", 10)).grid(
            row=3, column=0, columnspan=2, sticky="w", **pad)
        ttk.Label(root, textvariable=self._hint_var, wraplength=560).grid(
            row=4, column=0, columnspan=2, sticky="w", **pad)

        obs = ttk.LabelFrame(root, text="观测（EKF 底盘 / 关节编码器 / tf 末端）")
        obs.grid(row=5, column=0, columnspan=2, sticky="ew", **pad)
        self._obs_chassis_var = tk.StringVar(value=self._obs_chassis)
        self._obs_arm_var = tk.StringVar(value=self._obs_arm)
        self._obs_ee_odom_var = tk.StringVar(value=self._obs_ee_odom)
        self._obs_ee_mount_var = tk.StringVar(value=self._obs_ee_mount)
        mono = {"font": ("monospace", 10)}
        ttk.Label(obs, textvariable=self._obs_chassis_var, **mono).pack(anchor="w", padx=8, pady=2)
        ttk.Label(obs, textvariable=self._obs_arm_var, **mono).pack(anchor="w", padx=8, pady=2)
        ttk.Label(obs, textvariable=self._obs_ee_odom_var, **mono).pack(anchor="w", padx=8, pady=2)
        ttk.Label(obs, textvariable=self._obs_ee_mount_var, **mono).pack(anchor="w", padx=8, pady=2)

        chassis = ttk.LabelFrame(root, text="底盘（斜坡限加速，防翻）")
        chassis.grid(row=6, column=0, columnspan=2, sticky="ew", **pad)
        speeds = ttk.Frame(chassis, padding=6)
        speeds.pack(fill="x")
        self._v_scale, self._v_label = self._add_scale(
            speeds, 0, "v max", 0.03, 0.25, self._v_max, "m/s", self._on_vmax)
        self._w_scale, self._w_label = self._add_scale(
            speeds, 1, "ω max", 0.05, 0.8, self._w_max, "rad/s", self._on_wmax)
        self._a_scale, self._a_label = self._add_scale(
            speeds, 2, "线加速度", 0.05, 0.6, self._a_max, "m/s²", self._on_amax)
        self._al_scale, self._al_label = self._add_scale(
            speeds, 3, "角加速度", 0.1, 1.5, self._alpha_max, "rad/s²", self._on_alphamax)

        body = ttk.Frame(chassis, padding=6)
        body.pack()
        pad_fr = ttk.Frame(body)
        pad_fr.grid(row=0, column=0, padx=8)
        ttk.Label(pad_fr, text="摇杆：上下前进，左右转").pack()
        self._canvas = tk.Canvas(pad_fr, width=200, height=200, bg="#1e1e1e",
                                 highlightthickness=0)
        self._canvas.pack()
        self._canvas.create_oval(16, 16, 184, 184, outline="#666")
        self._canvas.create_line(100, 16, 100, 184, fill="#444")
        self._canvas.create_line(16, 100, 184, 100, fill="#444")
        self._knob = self._canvas.create_oval(86, 86, 114, 114, fill="#4ea1ff", outline="")
        self._canvas.bind("<ButtonPress-1>", self._on_stick)
        self._canvas.bind("<B1-Motion>", self._on_stick)
        self._canvas.bind("<ButtonRelease-1>", self._on_stick_up)

        grid = ttk.Frame(body)
        grid.grid(row=0, column=1, padx=8)
        self._mk_hold(grid, "前进 W", "w").grid(row=0, column=1, padx=3, pady=3)
        self._mk_hold(grid, "左转 A", "a").grid(row=1, column=0, padx=3, pady=3)
        ttk.Button(grid, text="停底盘", command=self._stop_chassis).grid(
            row=1, column=1, padx=3, pady=3)
        self._mk_hold(grid, "右转 D", "d").grid(row=1, column=2, padx=3, pady=3)
        self._mk_hold(grid, "后退 S", "s").grid(row=2, column=1, padx=3, pady=3)
        self._mk_hold(grid, "原地左转 Q", "q").grid(row=3, column=0, padx=3, pady=3)
        self._mk_hold(grid, "原地右转 E", "e").grid(row=3, column=2, padx=3, pady=3)

        self._latch_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(chassis, text="锁定巡航（斜坡到目标后保持，点「停底盘」才减速）",
                        variable=self._latch_var,
                        command=self._on_latch).pack(anchor="w", padx=8, pady=4)

        ee = ttk.LabelFrame(root, text="末端轨迹（验证控制模式）")
        ee.grid(row=7, column=0, columnspan=2, sticky="ew", **pad)
        opts = ttk.Frame(ee, padding=6)
        opts.pack(fill="x")
        ttk.Label(opts, text="位移 m").pack(side="left")
        self._dist = tk.DoubleVar(value=0.06)
        ttk.Spinbox(opts, from_=0.02, to=0.20, increment=0.01, width=6,
                    textvariable=self._dist).pack(side="left", padx=4)
        ttk.Label(opts, text="时间 s").pack(side="left")
        self._dur = tk.DoubleVar(value=5.0)
        ttk.Spinbox(opts, from_=2.0, to=12.0, increment=0.5, width=6,
                    textvariable=self._dur).pack(side="left", padx=4)

        row = ttk.Frame(ee, padding=6)
        row.pack(fill="x")
        ttk.Button(row, text="世界系 +X（全身 kWholeBody）",
                   command=lambda: self._send_ee(TaskSpacePoint.EE_ODOM, 1, 0, 0)).pack(
            side="left", padx=3, pady=2)
        ttk.Button(row, text="世界系 −X",
                   command=lambda: self._send_ee(TaskSpacePoint.EE_ODOM, -1, 0, 0)).pack(
            side="left", padx=3, pady=2)
        row2 = ttk.Frame(ee, padding=6)
        row2.pack(fill="x")
        ttk.Button(row2, text="臂座 +X（kArmRelative）",
                   command=lambda: self._send_ee(TaskSpacePoint.EE_MOUNT, 1, 0, 0)).pack(
            side="left", padx=3, pady=2)
        ttk.Button(row2, text="臂座 +Z 抬",
                   command=lambda: self._send_ee(TaskSpacePoint.EE_MOUNT, 0, 0, 1)).pack(
            side="left", padx=3, pady=2)
        ttk.Button(row2, text="车体 +X（base）",
                   command=lambda: self._send_ee(TaskSpacePoint.EE_BASE, 1, 0, 0)).pack(
            side="left", padx=3, pady=2)
        row3 = ttk.Frame(ee, padding=6)
        row3.pack(fill="x")
        ttk.Button(row3, text="清除轨迹 → Hold", command=self._clear_traj).pack(
            side="left", padx=3, pady=2)
        ttk.Label(row3, text="开车同时点臂座/车体轨迹 = kTeleopRelative").pack(
            side="left", padx=8)

    def _add_scale(self, parent, row, name, lo, hi, value, unit, cb):
        ttk.Label(parent, text=name).grid(row=row, column=0, sticky="w")
        scale = ttk.Scale(parent, from_=lo, to=hi, value=value, command=cb, length=240)
        scale.grid(row=row, column=1, sticky="ew", padx=6)
        label = ttk.Label(parent, width=12, text=f"{value:.2f} {unit}")
        label.grid(row=row, column=2, sticky="w")
        return scale, label

    def _mk_hold(self, parent, text, key):
        b = ttk.Button(parent, text=text)
        b.bind("<ButtonPress-1>", lambda _e, k=key: self._keys.add(k))
        b.bind("<ButtonRelease-1>", lambda _e, k=key: self._keys.discard(k))
        return b

    def _on_vmax(self, _=None):
        self._v_max = float(self._v_scale.get())
        self._v_label.config(text=f"{self._v_max:.2f} m/s")

    def _on_wmax(self, _=None):
        self._w_max = float(self._w_scale.get())
        self._w_label.config(text=f"{self._w_max:.2f} rad/s")

    def _on_amax(self, _=None):
        self._a_max = float(self._a_scale.get())
        self._a_label.config(text=f"{self._a_max:.2f} m/s²")

    def _on_alphamax(self, _=None):
        self._alpha_max = float(self._al_scale.get())
        self._al_label.config(text=f"{self._alpha_max:.2f} rad/s²")

    def _on_latch(self):
        self._latch = bool(self._latch_var.get())
        if not self._latch:
            self._latched = (0.0, 0.0)

    def _on_key(self, event):
        key = event.keysym.lower()
        moving = ("w", "a", "s", "d", "q", "e")
        if event.type == tk.EventType.KeyPress:
            if key in ("space", "escape"):
                self._stop_chassis()
                return
            if key in moving:
                self._keys.add(key)
        elif event.type == tk.EventType.KeyRelease:
            if key in moving:
                self._keys.discard(key)

    def _on_stick(self, event):
        x = (event.x - 100) / 84.0
        y = (100 - event.y) / 84.0
        n = math.hypot(x, y)
        if n > 1.0:
            x /= n
            y /= n
        self._stick = (y, -x)
        self._canvas.coords(self._knob, 100 + x * 84 - 14, 100 - y * 84 - 14,
                            100 + x * 84 + 14, 100 - y * 84 + 14)

    def _on_stick_up(self, _event):
        self._stick = (0.0, 0.0)
        self._canvas.coords(self._knob, 86, 86, 114, 114)

    def _target_vw(self):
        v = 0.0
        w = 0.0
        if "w" in self._keys:
            v += self._v_max
        if "s" in self._keys:
            v -= self._v_max
        if "a" in self._keys or "q" in self._keys:
            w += self._w_max
        if "d" in self._keys or "e" in self._keys:
            w -= self._w_max
        v += self._stick[0] * self._v_max
        w += self._stick[1] * self._w_max
        v = clamp(v, -self._v_max, self._v_max)
        w = clamp(w, -self._w_max, self._w_max)
        moving = abs(v) > 1e-4 or abs(w) > 1e-4
        if moving:
            self._latched = (v, w)
            return v, w
        if self._latch and (abs(self._latched[0]) > 1e-4 or abs(self._latched[1]) > 1e-4):
            return self._latched
        return 0.0, 0.0

    def _stop_chassis(self):
        self._keys.clear()
        self._stick = (0.0, 0.0)
        self._latched = (0.0, 0.0)
        self._latch_var.set(False)
        self._latch = False
        self._canvas.coords(self._knob, 86, 86, 114, 114)

    def _tick(self):
        tgt_v, tgt_w = self._target_vw()
        self._v_cmd = slew(self._v_cmd, tgt_v, self._a_max * DT)
        self._w_cmd = slew(self._w_cmd, tgt_w, self._alpha_max * DT)
        moving = abs(self._v_cmd) > 1e-4 or abs(self._w_cmd) > 1e-4
        want = abs(tgt_v) > 1e-4 or abs(tgt_w) > 1e-4
        self._sending = moving or want
        if self._sending:
            msg = Twist()
            msg.linear.x = self._v_cmd
            msg.angular.z = self._w_cmd
            self._cmd_pub.publish(msg)
        self._update_ee_obs()

    def _lookup(self, parent, child):
        return self._tf.lookup_transform(parent, child, Time())

    def _send_ee(self, ee_frame, ux, uy, uz):
        dist = float(self._dist.get())
        dur = max(1.0, float(self._dur.get()))
        n = math.sqrt(ux * ux + uy * uy + uz * uz) or 1.0
        dx, dy, dz = dist * ux / n, dist * uy / n, dist * uz / n

        parent = {
            TaskSpacePoint.EE_ODOM: "odom",
            TaskSpacePoint.EE_BASE: "base_link",
            TaskSpacePoint.EE_MOUNT: "arm_base_link",
        }[ee_frame]
        try:
            tf = self._lookup(parent, "brush_tip")
        except TransformException as exc:
            self._hint = f"查 tf {parent}→brush_tip 失败：{exc}"
            return

        start = tf_to_pose(tf)
        end = shifted(start, dx, dy, dz)
        vx, vy, vz = dx / dur, dy / dur, dz / dur

        def point(t, pose, moving):
            p = TaskSpacePoint()
            p.time_from_start = to_dur(t)
            p.ee_frame = ee_frame
            p.ee_pose = pose
            if moving:
                p.ee_twist.linear.x = vx
                p.ee_twist.linear.y = vy
                p.ee_twist.linear.z = vz
            p.chassis_mode = (TaskSpacePoint.CHASSIS_FREE
                              if ee_frame == TaskSpacePoint.EE_ODOM
                              else TaskSpacePoint.CHASSIS_TRACK_POSE)
            p.surface_normal.z = 1.0
            return p

        traj = TaskSpaceTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = parent
        traj.points = [point(0.0, start, True), point(dur, end, False)]
        self._traj_pub.publish(traj)
        self._traj_frame = ee_frame
        self._traj_note = (
            f"已发 {FRAME_NAME[ee_frame]}  {parent} Δ=[{dx:+.3f},{dy:+.3f},{dz:+.3f}] m  {dur:.1f}s"
        )
        self._hint = (
            "全身：车会帮够。相对：车应钉住。开车同时发相对轨迹 → kTeleopRelative。"
        )

    def _clear_traj(self):
        traj = TaskSpaceTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        self._traj_pub.publish(traj)
        self._traj_frame = None
        self._traj_note = "已清轨迹，无遥控后回 Hold"
        self._stop_chassis()

    def _on_status(self, msg):
        qp = QP_NAME.get(int(msg.qp_status), str(msg.qp_status))
        e = msg.ee_error
        self._status = (
            f"QP {qp}  iter={msg.qp_iterations}  {msg.qp_solve_time_us:.0f} µs   "
            f"ee [{e[0]:+.3f} {e[1]:+.3f} {e[2]:+.3f}] m  "
            f"v_cmd={msg.chassis_v_cmd:+.2f}  ω_cmd={msg.chassis_omega_cmd:+.2f}"
        )

    def _update_ee_obs(self):
        try:
            self._obs_ee_odom = fmt_pose("末端 odom", tf_to_pose(
                self._lookup("odom", "brush_tip")))
        except TransformException:
            self._obs_ee_odom = "末端 odom：等待 tf odom→brush_tip"
        try:
            self._obs_ee_mount = fmt_pose("末端 臂座", tf_to_pose(
                self._lookup("arm_base_link", "brush_tip")))
        except TransformException:
            self._obs_ee_mount = "末端 臂座：等待 tf arm_base_link→brush_tip"

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        self._odom = f"odom  x={p.x:+.2f}  y={p.y:+.2f}  yaw={math.degrees(yaw):+.1f}°"
        self._obs_chassis = (
            f"底盘  v={v:+.3f} m/s  ω={omega:+.3f} rad/s   "
            f"位姿 x={p.x:+.3f} y={p.y:+.3f} yaw={math.degrees(yaw):+.1f}°"
        )

    def _on_joints(self, msg):
        by_name = dict(zip(msg.name, msg.position))
        parts = []
        for i, name in enumerate(ARM_JOINTS, start=1):
            if name not in by_name:
                parts.append(f"J{i}=?")
                continue
            q = by_name[name]
            if name in PRISMATIC:
                parts.append(f"J{i}={q:+.3f}m")
            else:
                parts.append(f"J{i}={math.degrees(q):+.1f}°")
        self._obs_arm = "臂  " + "  ".join(parts)

    def _mode_text(self):
        driving = self._sending
        frame = self._traj_frame
        if driving and frame in (TaskSpacePoint.EE_BASE, TaskSpacePoint.EE_MOUNT):
            return "模式 kTeleopRelative（开车 + 相对臂）"
        if driving:
            return "模式 kChassisTeleop（开车，臂回舒适构型）"
        if frame == TaskSpacePoint.EE_ODOM:
            return "模式 kWholeBody（世界系工具，底盘可帮）"
        if frame in (TaskSpacePoint.EE_BASE, TaskSpacePoint.EE_MOUNT):
            return "模式 kArmRelative（相对轨迹，底盘钉住）"
        return "模式 kHold（钉车位 + 世界系工具）"

    def _refresh(self):
        self._status_var.set(self._status)
        self._odom_var.set(self._odom)
        if self._sending:
            self._cmd_var.set(
                f"底盘斜坡  v={self._v_cmd:+.2f} m/s  ω={self._w_cmd:+.2f} rad/s   {self._mode_text()}"
            )
        else:
            self._cmd_var.set(f"底盘未发送   {self._mode_text()}")
        self._traj_var.set(self._traj_note)
        self._hint_var.set(self._hint)
        self._obs_chassis_var.set(self._obs_chassis)
        self._obs_arm_var.set(self._obs_arm)
        self._obs_ee_odom_var.set(self._obs_ee_odom)
        self._obs_ee_mount_var.set(self._obs_ee_mount)
        self._root.after(50, self._refresh)

    def _on_close(self):
        self._stop_chassis()
        self._root.destroy()

    def run(self):
        self._root.mainloop()


def main(args=None):
    remove_ros_args(args if args is not None else sys.argv)
    rclpy.init(args=args)
    node = TeleopGui()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
