import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_arm(robot_ip: str, ee_id_val: str, is_left: bool):

    if robot_ip == "":
        return []

    # path to packages
    bringup_pkg = os.path.join(get_package_share_directory("fr3_bringup"))
    description_pkg = os.path.join(get_package_share_directory("fr3_description"))
    controllers_pkg = os.path.join(get_package_share_directory("fr3_controllers"))
    xacro_file = os.path.join(description_pkg, "robots", "fr3", "fr3.urdf.xacro")

    # setup variables
    controllers_param_file = os.path.join(
        controllers_pkg,
        "config",
        ("fr3_left" if is_left else "fr3_right") + "_controllers.yaml",
    )
    arm_id = "fr3_left" if is_left else "fr3_right"
    controller_manager_arg = f"/{arm_id}/controller_manager"
    left_robot_ip = robot_ip if is_left else ""
    right_robot_ip = "" if is_left else robot_ip
    load_gripper_val = "true" if ee_id_val else "false"
    gripper_executable = (
        "fake_gripper_state_publisher.py"
        if (robot_ip == "fake")
        else "franka_gripper_node"
    )
    gripper_joint_names = [f"{arm_id}_finger_joint1", f"{arm_id}_finger_joint2"]

    # robot urdf
    robot_urdf = xacro.process_file(
        xacro_file,
        mappings={
            "hand": load_gripper_val,
            "ros2_control": "true",
            "gazebo": "false",
            "ee_id": ee_id_val,
            "left_robot_ip": left_robot_ip,
            "right_robot_ip": right_robot_ip,
        },
    )

    # setup robot_description params for this arm
    robot_description_params = {
        "robot_description": robot_urdf.toxml(),
        "use_sim_time": False,
    }

    launch_list = []

    # append controller manager
    launch_list.append(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=f"/{arm_id}",
            parameters=[robot_description_params, controllers_param_file],
            output={"stdout": "screen", "stderr": "screen"},
        )
    )

    # append joint state broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", controller_manager_arg],
    )
    launch_list.append(joint_state_broadcaster)

    if load_gripper_val == "true":

        # append gripper controller
        launch_list.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=["gripper_controller", "-c", controller_manager_arg],
                    ),
                )
            )
        )

    # append robot state publisher node
    launch_list.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=arm_id,
            parameters=[robot_description_params],
            output="screen",
        )
    )

    return launch_list


def launch_setup(context, *args, **kwargs):

    left_robot_ip = context.perform_substitution(LaunchConfiguration("left_robot_ip"))
    right_robot_ip = context.perform_substitution(LaunchConfiguration("right_robot_ip"))
    ee_id_val = context.perform_substitution(LaunchConfiguration("ee_id"))
    rviz_val = context.perform_substitution(LaunchConfiguration("rviz")) == "true"

    launch_list = load_arm(left_robot_ip, ee_id_val, True) + load_arm(
        right_robot_ip, ee_id_val, False
    )

    rviz_file = os.path.join(
        get_package_share_directory("fr3_description"), "rviz", "bringup.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
    )

    if rviz_val:
        launch_list.append(rviz_node)

    return launch_list


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            name="left_robot_ip",
            default_value="",
            description="IP address of the left robot",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="right_robot_ip",
            default_value="",
            description="IP address of the right robot",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="rviz", default_value="true", description="Launch rviz"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="ee_id", default_value="", description="Gripper type"
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )