from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare separate arguments for left and right arms.
    left_robot_ip_arg = DeclareLaunchArgument(
        'left_robot_ip',
        description='Hostname or IP address of the left robot.'
    )
    right_robot_ip_arg = DeclareLaunchArgument(
        'right_robot_ip',
        description='Hostname or IP address of the right robot.'
    )
    left_arm_id_arg = DeclareLaunchArgument(
        'left_arm_id',
        default_value='left_fr3',
        description='ID for the left arm (used for namespacing and topic names).'
    )
    right_arm_id_arg = DeclareLaunchArgument(
        'right_arm_id',
        default_value='right_fr3',
        description='ID for the right arm (used for namespacing and topic names).'
    )
    # Other shared parameters.
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Visualize the robot in RViz'
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware'
    )
    fake_sensor_commands_arg = DeclareLaunchArgument(
        'fake_sensor_commands',
        default_value='false',
        description='Fake sensor commands. Only valid when use_fake_hardware is true'
    )
    load_gripper_arg = DeclareLaunchArgument(
        'load_gripper',
        default_value='true',
        description='Use Franka Gripper as an end-effector'
    )

    # LaunchConfigurations for ease of use.
    left_robot_ip = LaunchConfiguration('left_robot_ip')
    right_robot_ip = LaunchConfiguration('right_robot_ip')
    left_arm_id = LaunchConfiguration('left_arm_id')
    right_arm_id = LaunchConfiguration('right_arm_id')
    use_rviz = LaunchConfiguration('use_rviz')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    load_gripper = LaunchConfiguration('load_gripper')

    # Include the bringup for the left arm.
    left_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("franka_bringup"),
                "launch",
                "franka.launch.py"
            ])
        ]),
        launch_arguments={
            'robot_ip': left_robot_ip,
            'arm_id': left_arm_id,
            'use_rviz': use_rviz,
            'use_fake_hardware': use_fake_hardware,
            'fake_sensor_commands': fake_sensor_commands,
            'load_gripper': load_gripper,
        }.items()
    )

    # Include the bringup for the right arm.
    right_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("franka_bringup"),
                "launch",
                "franka.launch.py"
            ])
        ]),
        launch_arguments={
            'robot_ip': right_robot_ip,
            'arm_id': right_arm_id,
            'use_rviz': use_rviz,
            'use_fake_hardware': use_fake_hardware,
            'fake_sensor_commands': fake_sensor_commands,
            'load_gripper': load_gripper,
        }.items()
    )

    # Spawn the controller for the left arm. Using PushRosNamespace
    left_controller_spawner = GroupAction([
        PushRosNamespace(LaunchConfiguration('left_arm_id')),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_impedance_controller"],
            output="screen"
        )
    ])

    # Spawn the controller for the right arm.
    right_controller_spawner = GroupAction([
        PushRosNamespace(LaunchConfiguration('right_arm_id')),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_impedance_controller"],
            output="screen"
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
        left_arm_launch,
        right_arm_launch,
        left_controller_spawner,
        right_controller_spawner,
    ])
