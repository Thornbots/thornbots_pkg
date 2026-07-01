import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg_share = get_package_share_directory("sentry_pkg")
    default_model_path = os.path.join(pkg_share, "urdf", "sentry.urdf.xacro")
    robot_description_config = xacro.process_file(default_model_path)
    robot_description_raw = robot_description_config.toxml()
    sllidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": LaunchConfiguration("lidar_serial_port"),
                "frame_id": "lidar",
                "serial_baudrate": 115200,
                "inverted": False,
                "angle_compensate": True,
            }
        ],
        remappings=[
            ('/scan', '/scan_raw')
        ],
        output="screen",
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{
            "robot_description": robot_description_raw,
            "source_list": ["/pose_translator/joint_states"]  # Blends external joint streams safely
        }],
    )
    pose_translator_node = Node(
        package="sentry_pkg",
        executable="pose_translator",
        name="pose_translator",
        output="screen",
        remappings=[
            ('/joint_states', '/pose_translator/joint_states')
        ]
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

    # Periodically serialize the pose graph (map + full SLAM state) every 60s.
    # Runs as a background shell loop so a crash/kill doesn't lose more than
    # ~1 minute of mapping progress. Uses serialize_map (not save_map) so the
    # result can be reloaded to continue mapping OR run localization later.
    map_autosave_process = ExecuteProcess(
        cmd=[
            "bash", "-c",
            [
                "while true; do "
                "sleep 60; "
                "ros2 service call /slam_toolbox/serialize_map "
                "slam_toolbox_msgs/srv/SerializePoseGraph "
                "\"{filename: '",
                LaunchConfiguration("map_save_path"),
                "'}\"; "
                "done"
            ]
        ],
        output="screen",
        name="map_autosave",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument(
                name="lidar_serial_port",
                default_value="/dev/ttyUSB0",
                description="Serial device path for the SLLIDAR. Use /dev/ttyUSB0 "
                            "when running standalone; inside the Isaac ROS container "
                            "the hotplug USB lidar is read via the /host-dev bind, "
                            "e.g. /host-dev/ttyUSB0.",
            ),
            DeclareLaunchArgument(
                name="enable_rviz",
                default_value="False",
                description="Launch rviz2. Off by default since the robot runs "
                            "headless; enable for bench debugging with a display.",
            ),
            DeclareLaunchArgument(
                name="map_save_path",
                default_value=os.path.join(pkg_share, "map", "sentry_map"),
                description="Path (without extension) that the map pose graph "
                            "is periodically serialized to, every 60 seconds.",
            ),
            robot_state_publisher_node,
            joint_state_publisher_node,
            sllidar_node,
            slam_toolbox_node,
            pose_translator_node,
            relocalize_node,
            map_autosave_process,
        ]
    )
