"""
Launch file for the RoboClaw ros2_control hardware interface.

Starts:
  1. robot_state_publisher  (URDF -> /robot_description + TF)
  2. controller_manager     (loads the hardware plugin)
  3. joint_state_broadcaster
  4. diagnostics_broadcaster
  5. diff_drive_controller
  6. rc_teleop_node          (RC per-wheel→cmd_vel with CH5 mode switch)

All parameters can be overridden from the command line:
  ros2 launch roboclaw_hardware roboclaw.launch.py tcp_host:=10.0.0.5
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("roboclaw_hardware")

    # -- Declare arguments --
    declared_args = [
        DeclareLaunchArgument("tcp_host", default_value="192.168.68.60"),
        DeclareLaunchArgument("tcp_port", default_value="8234"),
        DeclareLaunchArgument("address", default_value="0x80"),
        DeclareLaunchArgument("namespace", default_value=""),
    ]

    # -- Generate URDF from xacro --
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([pkg_share, "urdf", "roboclaw_diff_drive.urdf.xacro"]),
        " tcp_host:=", LaunchConfiguration("tcp_host"),
        " tcp_port:=", LaunchConfiguration("tcp_port"),
        " address:=",  LaunchConfiguration("address"),
    ])

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controller_config = PathJoinSubstitution(
        [pkg_share, "config", "diff_drive_controllers.yaml"])

    # -- Nodes --
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[robot_description, controller_config],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                    "--controller-manager-timeout", "30"],
        namespace=LaunchConfiguration("namespace"),
        output="screen",
    )

    diagnostics_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diagnostics_broadcaster",
                    "--controller-manager-timeout", "30"],
        namespace=LaunchConfiguration("namespace"),
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller",
                    "--controller-manager-timeout", "30"],
        namespace=LaunchConfiguration("namespace"),
        output="screen",
    )

    # -- RC Teleop (per-wheel tank + CH5 mode switch) --
    rc_teleop_node = Node(
        package="roboclaw_tcp_adapter",
        executable="rc_teleop_node",
        name="rc_teleop",
        output="screen",
        parameters=[{
            "mixing_mode": "tank",
            "max_wheel_speed": 4.5,
            "wheel_separation": 0.4,
            "left_topic": "/robot/motor_left",
            "right_topic": "/robot/motor_right",
            "mode_switch_topic": "/robot/rc_mode",
            "mode_switch_threshold": 0.5,
            "cmd_vel_topic": "/diff_drive_controller/cmd_vel",
            "estop_topic": "/robot/estop",
            "deadzone": 0.05,
            "publish_rate": 20.0,
        }],
    )

    # Chain: joint_state_broadcaster -> diagnostics_broadcaster -> diff_drive
    delayed_diagnostics = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diagnostics_broadcaster_spawner],
        )
    )

    delayed_diff_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diagnostics_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    return LaunchDescription(
        declared_args + [
            robot_state_publisher,
            controller_manager,
            joint_state_broadcaster_spawner,
            delayed_diagnostics,
            delayed_diff_drive,
            rc_teleop_node,
        ]
    )
