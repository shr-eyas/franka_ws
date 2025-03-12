from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def robot_description_spawner(context,
                              robot_ip,
                              arm_id,
                              use_fake_hardware,
                              fake_sensor_commands,
                              load_gripper):

    arm_id_str = context.perform_substitution(arm_id)

    robot_description = xacro.process_file(
        os.path.join(
            get_package_share_directory('franka_description'),
            'robots',
            arm_id_str,
            arm_id_str + '.urdf.xacro'
        ),
        mappings={
            'ros2_control': 'true',
            'arm_id': context.perform_substitution(arm_id),
            'robot_ip': context.perform_substitution(robot_ip),
            'hand': context.perform_substitution(load_gripper),
            'use_fake_hardware': context.perform_substitution(use_fake_hardware),
            'fake_sensor_commands': context.perform_substitution(fake_sensor_commands)
        }
    ).toprettyxml(indent='  ')

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare('franka_bringup'), 'config', 'controllers.yaml'])

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controllers_yaml, {'robot_description': robot_description, 'arm_id': arm_id}],
            remappings=[('joint_states', 'franka/joint_states')],
            output='screen',
            on_exit=Shutdown(),
        )
    ]


def generate_launch_description():

    # Declare launch arguments
    robot_ip = LaunchConfiguration("robot_ip")
    arm_id = LaunchConfiguration("arm_id")
    load_gripper = LaunchConfiguration("load_gripper")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    use_rviz = LaunchConfiguration("use_rviz")

    rviz_file = os.path.join(
        get_package_share_directory('franka_description'), 'rviz', 'visualize_franka.rviz')

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument("robot_ip", description="IP of the robot"),
        DeclareLaunchArgument("arm_id", default_value="fr3", description="Robot Arm ID"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("fake_sensor_commands", default_value="false"),
        DeclareLaunchArgument("load_gripper", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),

        # Robot description nodes
        OpaqueFunction(
            function=robot_description_spawner,
            args=[robot_ip, arm_id, use_fake_hardware, fake_sensor_commands, load_gripper]
        ),

        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{
                'source_list': ['franka/joint_states', 'franka_gripper/joint_states'],
                'rate': 30
            }],
        ),

        # Controller spawners
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen"
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["franka_robot_state_broadcaster"],
            parameters=[{"arm_id": arm_id}],
            output="screen",
            condition=UnlessCondition(use_fake_hardware)
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_impedance_controller"],
            output="screen"
        ),

        # Gripper (conditional)
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'
            ]),
            launch_arguments={
                'robot_ip': robot_ip,
                'use_fake_hardware': use_fake_hardware
            }.items(),
            condition=IfCondition(load_gripper)
        ),

        # RViz (conditional)
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['--display-config', rviz_file],
            condition=IfCondition(use_rviz)
        )
    ])