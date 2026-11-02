"""Record the topics needed to replay and plot an experiment (mcap, one folder per run).

    ros2 launch wbc record.launch.py [name:=wall_test]
"""
import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

TOPICS = [
    "/joint_states", "/tf", "/tf_static", "/odom",
    "/imu_sensor_broadcaster/imu", "/ft_sensor_broadcaster/wrench",
    "/wbc_controller/reference", "/wbc_controller/cmd_vel",
    "/wbc_controller/status", "/wbc_controller/contact",
    "/wbc_controller/ft_status", "/wbc_controller/wrench_compensated",
]


def record(context):
    name = LaunchConfiguration("name").perform(context)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = f"bags/{stamp}_{name}" if name else f"bags/{stamp}"
    return [ExecuteProcess(cmd=["ros2", "bag", "record", "-s", "mcap", "-o", out, *TOPICS],
                           output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("name", default_value=""),
        OpaqueFunction(function=record),
    ])
