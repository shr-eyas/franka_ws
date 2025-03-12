#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments shared by both arms
    left_robot_ip_arg = DeclareLaunchArgument(
        'left_robot_ip',
        default_value='192.168.1.12',
        description='Hostname or IP address of the left robot.'
    )
    right_robot_ip_arg = DeclareLaunchArgument(
        'right_robot_ip',
        default_value='192.168.1.11',
        description='Hostname or IP address of the right robot.'
    )
    left_arm_id_arg = DeclareLaunchArgument(
        'left_arm_id',
        default_value='fr3_left',
        description='ID of the left arm.'
    )
    right_arm_id_arg = DeclareLaunchArgument(
        'right_arm_id',
        default_value='fr3_right',
        description='ID of the right arm.'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Visualize the robots in Rviz'
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware'
    )
    fake_sensor_commands_arg = DeclareLaunchArgument(
        'fake_sensor_commands',
        default_value='false',
        description='Fake sensor commands. Only valid when fake hardware is true'
    )
    load_gripper_arg = DeclareLaunchArgument(
        'load_gripper',
        default_value='true',
        description='Use Franka Gripper as an end-effector; otherwise, the robot is loaded without an end-effector.'
    )

    # Launch configurations
    left_robot_ip = LaunchConfiguration('left_robot_ip')
    right_robot_ip = LaunchConfiguration('right_robot_ip')
    left_arm_id = LaunchConfiguration('left_arm_id')
    right_arm_id = LaunchConfiguration('right_arm_id')
    use_rviz = LaunchConfiguration('use_rviz')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    load_gripper = LaunchConfiguration('load_gripper')

    # Include the dual arm launch (which spawns both robots)
    dual_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution(
                [FindPackageShare('franka_bringup'), 'launch', 'dual_arm.launch.py']
            )
        ]),
        launch_arguments={
            'left_robot_ip': left_robot_ip,
            'right_robot_ip': right_robot_ip,
            'left_arm_id': left_arm_id,
            'right_arm_id': right_arm_id,
            'use_rviz': use_rviz,
            'use_fake_hardware': use_fake_hardware,
            'fake_sensor_commands': fake_sensor_commands,
            'load_gripper': load_gripper,
        }.items()
    )

    # Spawn the joint impedance controller for the left arm in its namespace
    left_impedance_spawner = GroupAction([
        PushRosNamespace('fr3_left'),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_impedance_controller"],
            output="screen",
        )
    ])

    # Spawn the joint impedance controller for the right arm in its namespace
    right_impedance_spawner = GroupAction([
        PushRosNamespace('fr3_right'),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_impedance_controller"],
            output="screen",
        )
    ])

    return LaunchDescription([
        left_robot_ip_arg,
        right_robot_ip_arg,
        left_arm_id_arg,
        right_arm_id_arg,
        use_rviz_arg,
        use_fake_hardware_arg,
        fake_sensor_commands_arg,
        load_gripper_arg,
        dual_arm_launch,
        left_impedance_spawner,
        right_impedance_spawner,
    ])