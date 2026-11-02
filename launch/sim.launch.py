"""MuJoCo simulation + ros2_control + WBC.

    ros2 launch wbc sim.launch.py [wbc:=false] [rerun:=true] [foxglove:=false] [rviz:=false]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def spawner(name, condition=None):
    return Node(package="controller_manager", executable="spawner",
                arguments=[name, "--controller-manager", "/controller_manager"],
                condition=condition)


def rerun_node(context, *args, **kwargs):
    del args, kwargs
    if LaunchConfiguration("rerun").perform(context) != "true":
        return []
    arguments = []
    save = LaunchConfiguration("rerun_save").perform(context)
    if save:
        arguments += ["--save", save]
    return [Node(package="wbc", executable="rerun_view.py", arguments=arguments, output="screen")]


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("wbc"), "urdf", "robot.urdf.xacro"])
    controllers = PathJoinSubstitution(
        [FindPackageShare("wbc"), "controllers.yaml"])
    robot_description = {"robot_description": ParameterValue(
        Command(["xacro ", xacro_file, " use_sim:=true"]), value_type=str)}

    return LaunchDescription([
        DeclareLaunchArgument("wbc", default_value="true"),
        DeclareLaunchArgument("rerun", default_value="true"),
        DeclareLaunchArgument("rerun_save", default_value=""),
        DeclareLaunchArgument("foxglove", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[robot_description]),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[robot_description, controllers], output="screen"),

        spawner("joint_state_broadcaster"),
        spawner("imu_sensor_broadcaster"),
        spawner("ft_sensor_broadcaster"),
        spawner("wbc_controller", IfCondition(LaunchConfiguration("wbc"))),
        OpaqueFunction(function=rerun_node),

        Node(package="foxglove_bridge", executable="foxglove_bridge",
             condition=IfCondition(LaunchConfiguration("foxglove"))),
        Node(package="rviz2", executable="rviz2",
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
