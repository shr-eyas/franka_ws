import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _robot_description_dependent_nodes_spawner(
    context: LaunchContext,
    namespace,
    robot_ip,
    arm_id,
    use_fake_hardware,
    fake_sensor_commands,
    load_gripper,
):

    # Resolve all substitutions at launch time
    namespace_str = context.perform_substitution(namespace)
    robot_ip_str = context.perform_substitution(robot_ip)
    arm_id_str = context.perform_substitution(arm_id)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(fake_sensor_commands)
    load_gripper_str = context.perform_substitution(load_gripper)

    # Path to Franka's URDF xacro
    franka_xacro_filepath = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        "fr3",
        "fr3.urdf.xacro",
    )

    # Generate the robot description from xacro
    robot_description = xacro.process_file(
        franka_xacro_filepath,
        mappings={
            "ros2_control": "true",
            "arm_id": arm_id_str,
            "robot_ip": robot_ip_str,
            "hand": load_gripper_str,
            "use_fake_hardware": use_fake_hardware_str,
            "fake_sensor_commands": fake_sensor_commands_str,
        },
    ).toprettyxml(indent="  ")

    # Path to joint limits file
    joint_limits_filepath = os.path.join(
        get_package_share_directory("franka_description"),
        "config",
        "joint_limits.yaml",
    )

    # Path to the controller configuration
    franka_controllers = PathJoinSubstitution(
        [FindPackageShare("fr3_controllers"), "config", "controllers.yaml"]
    )

    # Return the list of Nodes to be spawned once the robot description is known
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace_str,
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=namespace_str,
            parameters=[
                franka_controllers,
                {"robot_description": robot_description},
                {"arm_id": arm_id_str},
                {"load_gripper": load_gripper_str},
                {"joint_limits": joint_limits_filepath},
            ],
            remappings=[("joint_states", "franka/joint_states")],
            output={"stdout": "screen", "stderr": "screen"},
            on_exit=Shutdown(),
        ),
    ]

def generate_launch_description():

    namespace_parameter_name = "namespace"
    robot_ip_parameter_name = "robot_ip"
    arm_id_parameter_name = "arm_id"
    load_gripper_parameter_name = "load_gripper"
    use_fake_hardware_parameter_name = "use_fake_hardware"
    fake_sensor_commands_parameter_name = "fake_sensor_commands"
    use_rviz_parameter_name = "use_rviz"

    # Create LaunchConfigurations
    namespace = LaunchConfiguration(namespace_parameter_name)
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    arm_id = LaunchConfiguration(arm_id_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    # RViz configuration
    rviz_file = os.path.join(
        get_package_share_directory("franka_description"), "rviz", "visualize_franka.rviz"
    )

    # Opaque function to handle dynamic URDF generation and node spawning
    robot_description_dependent_nodes_spawner_opaque_function = OpaqueFunction(
        function=_robot_description_dependent_nodes_spawner,
        args=[
            namespace,
            robot_ip,
            arm_id,
            use_fake_hardware,
            fake_sensor_commands,
            load_gripper,
        ],
    )

    launch_description = LaunchDescription(
        [
            DeclareLaunchArgument(
                namespace_parameter_name,
                default_value="",
                description="Namespace to isolate multiple robot instances.",
            ),
            DeclareLaunchArgument(
                robot_ip_parameter_name,
                description="Hostname or IP address of the robot.",
            ),
            DeclareLaunchArgument(
                arm_id_parameter_name,
                default_value="fr3",
                description="ID of the type of arm used (e.g., fr3).",
            ),
            DeclareLaunchArgument(
                use_rviz_parameter_name,
                default_value="false",
                description="Visualize the robot in RViz.",
            ),
            DeclareLaunchArgument(
                use_fake_hardware_parameter_name,
                default_value="false",
                description="Use fake hardware.",
            ),
            DeclareLaunchArgument(
                fake_sensor_commands_parameter_name,
                default_value="false",
                description="Fake sensor commands (only valid when use_fake_hardware is true).",
            ),
            DeclareLaunchArgument(
                load_gripper_parameter_name,
                default_value="true",
                description="Use Franka Gripper as an end-effector. Otherwise, load robot without an end-effector.",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                namespace=namespace,
                name="joint_state_publisher",
                parameters=[
                    {
                        "source_list": [
                            "franka/joint_states",
                            "franka_gripper/joint_states",
                        ],
                        "rate": 30,
                    }
                ],
            ),
            # Dynamically generate URDF and bring up ros2_control_node
            robot_description_dependent_nodes_spawner_opaque_function,
            # Spawn joint_state_broadcaster
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
            # Spawn franka_robot_state_broadcaster (only if not using fake hardware)
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=["franka_robot_state_broadcaster"],
                parameters=[{"arm_id": arm_id}],
                output="screen",
                condition=UnlessCondition(use_fake_hardware),
            ),
            # Optionally include the Franka gripper launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [FindPackageShare("franka_gripper"), "launch", "gripper.launch.py"]
                        )
                    ]
                ),
                launch_arguments={
                    robot_ip_parameter_name: robot_ip,
                    use_fake_hardware_parameter_name: use_fake_hardware,
                    namespace_parameter_name: namespace,
                }.items(),
                condition=IfCondition(load_gripper),
            ),
            # Optionally launch RViz
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_file],
                condition=IfCondition(use_rviz),
            ),
            # Finally, spawn your main joint impedance controller
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=["joint_velocity_controller"],
                output="screen",
            ),
        ]
    )

    return launch_description