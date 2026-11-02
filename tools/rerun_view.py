#!/usr/bin/env python3
"""把话题记进 Rerun。跑在控制线程外面，不进 500 Hz 回路。

    ros2 run wbc rerun_view.py [--no-spawn] [--save bags/run.rrd]
"""
import argparse

import rclpy
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

import rerun as rr
import rerun.blueprint as rrb
from wbc.msg import ContactState, WbcStatus

EE_NAMES = ("x", "y", "z", "axis_x", "axis_y")
AXES_FRAMES = ("base_link", "brush_tip")


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def blueprint():
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="/", name="机器人"),
            rrb.Vertical(
                rrb.TimeSeriesView(origin="/plots/ee", name="刷头误差"),
                rrb.TimeSeriesView(origin="/plots/base", name="底盘"),
                rrb.TimeSeriesView(origin="/plots/force", name="力"),
                rrb.TimeSeriesView(origin="/plots/timing", name="耗时 us"),
            ),
        ),
        collapse_panels=True,
    )


def log_pose(path, translation, rotation, parent, child):
    rr.log(path, rr.Transform3D(
        translation=translation,
        quaternion=rr.Quaternion(xyzw=rotation),
        parent_frame=parent,
        child_frame=child,
    ))


class RerunView(Node):
    def __init__(self):
        super().__init__("rerun_view")
        self._axes = set()

        def reliable(depth=10, durability=DurabilityPolicy.VOLATILE):
            # 每次新建。同一个 QoSProfile 交给多次 create_subscription 后，底层指针会被释放。
            return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                              durability=durability)

        self.create_subscription(WbcStatus, "/wbc_controller/status", self.on_status, reliable())
        self.create_subscription(ContactState, "/wbc_controller/contact", self.on_contact, reliable())
        self.create_subscription(WrenchStamped, "/wbc_controller/wrench_compensated",
                                 self.on_wrench, reliable())
        self.create_subscription(JointState, "/joint_states", self.on_joints, reliable())
        self.create_subscription(Odometry, "/odom", self.on_odom, reliable())
        self.create_subscription(TFMessage, "/tf", self.on_tf, reliable())
        self.create_subscription(TFMessage, "/tf_static", self.on_tf,
                                 reliable(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(WrenchStamped, "/ft_sensor_broadcaster/wrench",
                                 self.on_raw_wrench, qos_profile_sensor_data)

    def _time(self, stamp):
        t = stamp_s(stamp)
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        rr.set_time("ros", duration=t)

    def _axes_once(self, frame):
        if frame in self._axes or frame not in AXES_FRAMES:
            return
        self._axes.add(frame)
        rr.log(f"frames/{frame}/axes", rr.TransformAxes3D(0.25), static=True)

    def on_status(self, msg):
        self._time(msg.header.stamp)
        for name, value in zip(EE_NAMES, msg.ee_error):
            rr.log(f"plots/ee/{name}", rr.Scalars(float(value)))
        rr.log("plots/base/v_cmd", rr.Scalars(msg.base_v_cmd))
        rr.log("plots/base/omega_cmd", rr.Scalars(msg.base_omega_cmd))
        rr.log("plots/timing/qp_us", rr.Scalars(msg.qp_solve_time_us))
        rr.log("plots/timing/cycle_us", rr.Scalars(msg.cycle_time_us))

    def on_contact(self, msg):
        self._time(msg.header.stamp)
        rr.log("plots/force/normal", rr.Scalars(msg.normal_force))
        rr.log("plots/force/in_contact", rr.Scalars(1.0 if msg.in_contact else 0.0))
        force = msg.wrench_world.force
        for axis, value in zip("xyz", (force.x, force.y, force.z)):
            rr.log(f"plots/force/{axis}", rr.Scalars(value))

    def on_wrench(self, msg):
        self._time(msg.header.stamp)
        torque = msg.wrench.torque
        for axis, value in zip("xyz", (torque.x, torque.y, torque.z)):
            rr.log(f"plots/force/t{axis}", rr.Scalars(value))

    def on_raw_wrench(self, msg):
        self._time(msg.header.stamp)
        force = msg.wrench.force
        for axis, value in zip("xyz", (force.x, force.y, force.z)):
            rr.log(f"plots/raw_force/{axis}", rr.Scalars(value))

    def on_joints(self, msg):
        self._time(msg.header.stamp)
        for name, position in zip(msg.name, msg.position):
            rr.log(f"plots/joint/{name}", rr.Scalars(position))

    def on_odom(self, msg):
        self._time(msg.header.stamp)
        twist = msg.twist.twist
        rr.log("plots/base/v", rr.Scalars(twist.linear.x))
        rr.log("plots/base/omega", rr.Scalars(twist.angular.z))

    def on_tf(self, msg):
        for tf in msg.transforms:
            child = tf.child_frame_id.lstrip("/")
            parent = tf.header.frame_id.lstrip("/")
            self._time(tf.header.stamp)
            t = tf.transform.translation
            q = tf.transform.rotation
            log_pose(f"frames/{child}", [t.x, t.y, t.z], [q.x, q.y, q.z, q.w], parent, child)
            self._axes_once(child)


def series_style():
    for name in EE_NAMES:
        rr.log(f"plots/ee/{name}", rr.SeriesLines(names=name), static=True)
    for name in ("v", "omega", "v_cmd", "omega_cmd"):
        rr.log(f"plots/base/{name}", rr.SeriesLines(names=name), static=True)
    for name in ("normal", "in_contact", "x", "y", "z", "tx", "ty", "tz"):
        rr.log(f"plots/force/{name}", rr.SeriesLines(names=name), static=True)
    for name in ("qp_us", "cycle_us"):
        rr.log(f"plots/timing/{name}", rr.SeriesLines(names=name), static=True)


def main():
    parser = argparse.ArgumentParser(description="Log WBC topics to Rerun")
    parser.add_argument("--no-spawn", action="store_true")
    parser.add_argument("--save", default="", help="also write an .rrd recording")
    args = parser.parse_args()

    rr.init("wbc", spawn=not args.no_spawn, default_blueprint=blueprint())
    if args.save:
        rr.save(args.save)
    series_style()

    rclpy.init()
    node = RerunView()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
