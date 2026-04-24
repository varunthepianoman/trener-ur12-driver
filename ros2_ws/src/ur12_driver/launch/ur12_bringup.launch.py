"""
Bring up the UR12 driver with ros2_control.

Starts:
  1. ur_driver_node       — lifecycle node (dashboard services + action servers)
  2. robot_state_publisher — TF from URDF
  3. controller_manager   — loads Ur12HardwareInterface + controllers
  4. joint_state_broadcaster — publishes /joint_states from hardware interface
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("ur12_driver")
    urdf_path        = os.path.join(pkg, "urdf", "ur12.urdf")
    controllers_yaml = os.path.join(pkg, "config", "ur12_controllers.yaml")

    robot_host_arg = DeclareLaunchArgument(
        "robot_host", default_value="ursim",
        description="Hostname or IP of the UR robot / URSim")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # 1. UR driver lifecycle node (dashboard + action servers)
    ur_driver_node = Node(
        package="ur12_driver",
        executable="ur_driver_node",
        name="ur_driver_node",
        parameters=[{"robot_host": LaunchConfiguration("robot_host")}],
        output="screen",
    )

    # 2. Robot state publisher (TF from URDF)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # 3. Controller manager (loads hardware interface plugin)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml,
        ],
        output="screen",
    )

    # 4. Spawners — started after controller_manager is up
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager",
            "--inactive",  # activate manually; write() is scaffold only
        ],
        output="screen",
    )

    # Delay spawners 3 s to let controller_manager initialise
    delayed_spawners = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,
        ],
    )

    return LaunchDescription([
        robot_host_arg,
        ur_driver_node,
        robot_state_publisher,
        controller_manager,
        delayed_spawners,
    ])
