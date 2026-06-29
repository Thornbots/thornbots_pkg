import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("sentry_pkg")
    default_model_path = os.path.join(pkg_share, "urdf", "sentry.urdf.xacro")
    robot_description_config = xacro.process_file(default_model_path)
    robot_description_raw = robot_description_config.toxml()
    sllidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": "/dev/ttyUSB1",
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True,
            }
        ],
        output="screen",
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"robot_description": robot_description_raw}],
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_raw}],
    )
    slam_params_file = os.path.join(pkg_share, "config", "slam.yaml")    
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            robot_state_publisher_node,
            joint_state_publisher_node,
            sllidar_node,
            slam_toolbox_node,
        ]
    )
