#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
import xacro

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    IncludeLaunchDescription,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def robot_description_dependent_nodes_spawner(context, robot_ip, arm_id,
                                              use_fake_hardware,
                                              fake_sensor_commands,
                                              load_gripper):
    # Substitute launch arguments
    robot_ip_str = context.perform_substitution(robot_ip)
    arm_id_str = context.perform_substitution(arm_id)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(fake_sensor_commands)
    load_gripper_str = context.perform_substitution(load_gripper)

    # Build the path to the xacro file (e.g. robots/fr3_left/fr3_left.urdf.xacro)
    franka_xacro_filepath = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        arm_id_str,
        arm_id_str + '.urdf.xacro'
    )
    # Process the xacro file with mappings
    robot_description = xacro.process_file(
        franka_xacro_filepath,
        mappings={
            'ros2_control': 'true',
            'arm_id': arm_id_str,
            'robot_ip': robot_ip_str,
            'hand': load_gripper_str,
            'use_fake_hardware': use_fake_hardware_str,
            'fake_sensor_commands': fake_sensor_commands_str,
        }
    ).toprettyxml(indent='  ')

    dual_arm_controllers = PathJoinSubstitution(
        [FindPackageShare('franka_bringup'), 'config', 'dual_arm_controllers.yaml']
    )
    
    nodes = []
    # Robot state publisher node
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    ))
    # ros2_control node
    nodes.append(Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[dual_arm_controllers,
                    {'robot_description': robot_description},
                    {'arm_id': arm_id}],
        # arm_id/joint_states
        remappings=[('joint_states', 'franka/joint_states')],
        output={'stdout': 'screen', 'stderr': 'screen'},
        on_exit=Shutdown(),
    ))
    # Joint state publisher node
    nodes.append(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['franka/joint_states', 'franka_gripper/joint_states'],
                     'rate': 30}],
    ))
    # Spawn the joint state broadcaster
    nodes.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    ))
    # Spawn the robot state broadcaster (only if not using fake hardware)
    nodes.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=['franka_robot_state_broadcaster'],
        parameters=[{'arm_id': arm_id}],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
    ))
    # Optionally include the gripper launch (if load_gripper is true)
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution(
                [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py']
            )
        ]),
        launch_arguments={
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware
        }.items(),
        condition=IfCondition(load_gripper)
    ))
    return nodes

def generate_launch_description():
    # Declare launch arguments for left and right arms
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

    # Create a GroupAction for the left arm nodes under the "fr3_left" namespace
    left_group = GroupAction([
        PushRosNamespace('fr3_left'),
        OpaqueFunction(
            function=robot_description_dependent_nodes_spawner,
            args=[left_robot_ip, left_arm_id, use_fake_hardware, fake_sensor_commands, load_gripper]
        )
    ])
    # Create a GroupAction for the right arm nodes under the "fr3_right" namespace
    right_group = GroupAction([
        PushRosNamespace('fr3_right'),
        OpaqueFunction(
            function=robot_description_dependent_nodes_spawner,
            args=[right_robot_ip, right_arm_id, use_fake_hardware, fake_sensor_commands, load_gripper]
        )
    ])

    # Optionally include a single RViz2 node for visualizing both arms.
    # (Make sure your rviz config includes topics for both namespaces.)
    rviz_file = os.path.join(
        get_package_share_directory('franka_description'),
        'rviz',
        'dual_arm.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_file],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        left_robot_ip_arg,
        right_robot_ip_arg,
        left_arm_id_arg,
        right_arm_id_arg,
        use_rviz_arg,
        use_fake_hardware_arg,
        fake_sensor_commands_arg,
        load_gripper_arg,
        left_group,
        right_group,
        rviz_node,
    ])
