import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory("sentry_pkg")
    default_model_path = os.path.join(pkg_share, "urdf", "sentry.urdf.xacro")
    robot_description_config = xacro.process_file(default_model_path)
    robot_description_raw = robot_description_config.toxml()
    lidar_port_cfg = LaunchConfiguration("lidar_serial_port")
    sllidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            "channel_type": "serial",
            "serial_port": lidar_port_cfg,
            "frame_id": "lidar",
            "serial_baudrate": 115200,
            "inverted": False,
            "angle_compensate": True,
        }],
        output="screen",
    )
    pose_translator_node = Node(
        package="sentry_pkg",
        executable="pose_translator",
        name="pose_translator",
        output="screen",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description_raw,
        }],
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
    relocalize_node = Node(
        package="sentry_pkg",
        executable="slam_relocalize_publisher",
        name="slam_relocalize_publisher",
        output="screen",
        parameters=[{
            "map_frame": "map",
            "base_frame": "root",
            "publish_rate_hz": 1.0,
            "relocalize_topic": "/dji_serial_bridge/relocalize",
        }],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            DeclareLaunchArgument(
                name="lidar_serial_port",
                default_value="/dev/ttyUSB0",
                description="Serial device path for the SLLIDAR. Use /dev/ttyUSB0 "
                            "when running standalone; inside the Isaac ROS container "
                            "the hotplug USB lidar is read via the /host-dev bind, "
                            "e.g. /host-dev/ttyUSB0.",
            ),
            robot_state_publisher_node,
            sllidar_node,
            slam_toolbox_node,
            pose_translator_node,
            relocalize_node,
        ]
    )
