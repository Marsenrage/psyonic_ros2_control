#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "hand_name",
            default_value="Left_Hand",
            description="Name of the hand",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "device_name",
            default_value="/dev/ttyUSB0",
            description="Serial device path",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "baud_rate",
            default_value="460800",
            description="Serial baud rate",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_psyonic",
            default_value="false",
            description="use mock hardware interface instead of real hardware",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "timeout",
            default_value="1.0",
            description="Timeout for hardware communication in seconds",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "hardware_name",
            default_value="PsyonicHandHardware",
            description="Name of the hardware interface class to use for the Psyonic hand",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reply_model",
            default_value="1",
            description="1:[positions, currents, touch sensors], 2:[positions, velocities, touch sensors], 3:[positions, currents, velocities]",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "command_mode",
            default_value="POSITION",
            description=" [positions, velocities, effort]",
        )
    )

    # Initialize Arguments
    hand_name = LaunchConfiguration("hand_name")
    device_name = LaunchConfiguration("device_name")
    baud_rate = LaunchConfiguration("baud_rate")
    use_mock_psyonic = LaunchConfiguration("use_mock_psyonic")
    timeout = LaunchConfiguration("timeout")
    reply_model = LaunchConfiguration("reply_model")
    command_mode = LaunchConfiguration("command_mode")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("psyonic_description"), "urdf", "psyonic_standalone.urdf.xacro"]),
            " ",
            "hand_name:=",
            hand_name,
            " ",
            "use_mock_psyonic:=",
            use_mock_psyonic,
            " ",
            "device_name:=",
            device_name,
            " ",
            "baud_rate:=",
            baud_rate,
            " ",
            "timeout:=",
            timeout,
            " ",
            "reply_model:=",
            reply_model,
            " ",
            "command_mode:=",
            command_mode,
            " ",
        ]
    )
    robot_description_content = ParameterValue(
        robot_description_content, value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("psyonic_bring_up"),
            "config",
            "psyonic_controller.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="left_hand",
        parameters=[robot_description, robot_controllers],
        output="both",
        condition=UnlessCondition(use_mock_psyonic),
    )

    mock_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="left_hand",
        parameters=[robot_description, robot_controllers],
        output="both",
        condition=IfCondition(use_mock_psyonic),
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="left_hand",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="left_hand",
        arguments=["joint_state_broadcaster", "--controller-manager", "/left_hand/controller_manager"],
    )

    psyonic_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="left_hand",
        arguments=["psyonic_left_hand", "--controller-manager", "/left_hand/controller_manager"],
    )

    # Delay controller spawner after joint state broadcaster
    delay_psyonic_hand_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[psyonic_hand_controller_spawner],
        )
    )

    nodes = [
        control_node,
        mock_control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_psyonic_hand_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)