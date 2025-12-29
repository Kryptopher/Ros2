import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial port for Teensy/Arduino",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "baud_rate",
            default_value="115200",
            description="Serial baud rate",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "steps_per_rev",
            default_value="200",
            description="Steps per revolution (full steps)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "microsteps",
            default_value="8",
            description="Microstepping setting",
        )
    )

    # Initialize Arguments
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    steps_per_rev = LaunchConfiguration("steps_per_rev")
    microsteps = LaunchConfiguration("microsteps")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("stepper_jtc_control"), "urdf", "stepper.urdf.xacro"]
            ),
            " ",
            "serial_port:=",
            serial_port,
            " ",
            "baud_rate:=",
            baud_rate,
            " ",
            "steps_per_rev:=",
            steps_per_rev,
            " ",
            "microsteps:=",
            microsteps,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Controller configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("stepper_jtc_control"),
            "config",
            "stepper_controllers.yaml",
        ]
    )

    # Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Trajectory controller spawner
    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["stepper_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay trajectory controller spawner after joint state broadcaster
    delay_trajectory_controller_spawner_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[trajectory_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_trajectory_controller_spawner_after_joint_state_broadcaster,
    ]

    return LaunchDescription(declared_arguments + nodes)
