import os, sys

from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction


def generate_launch_description():
    pkg_share = get_package_share_directory("sentry_pkg")
    default_model_path = os.path.join(
        pkg_share, "description", "sentry_description.urdf"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz", "config.rviz")
    world_path = os.path.join(pkg_share, "world", "my_world.sdf")
    pkg_share = get_package_share_directory("sentry_pkg")
    xacro_file = os.path.join(pkg_share, "urdf", "sentry.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_raw = robot_description_config.toxml()

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"robot_description": robot_description_raw}],
    )
    sllidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": "/dev/ttyUSB1",  # Double-check your port!
                "frame_id": "laser",  # Matches the URDF link name
                "inverted": False,
                "angle_compensate": True,
            }
        ],
        output="screen",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_raw,
                    'use_sim_time': True,
        'publish_robot_description': True  # Force it to publish the topic Gazebo needs
        }],
    )
    ros_gz_sim = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "sentry",
            "-topic",
            "robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.1",
        ],
        output="screen",
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    hardware = LaunchDescription(
        [
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            # REAL HARDWARE
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            robot_state_publisher_node,
            joint_state_publisher_node,
            sllidar_node,
            rviz_node,
        ]
    )
    software = LaunchDescription(
        [
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("ros_gz_sim"),
                            "launch",
                            "gz_sim.launch.py",
                        )
                    ]
                ),
                launch_arguments={"gz_args": f"-r {world_path}"}.items(),
            ),
            joint_state_publisher_node,
            robot_state_publisher_node,
            rviz_node,
            ros_gz_bridge,
            ros_gz_sim,
        ]
    )
    return software
