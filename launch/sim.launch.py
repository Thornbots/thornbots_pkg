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
    world_path = os.path.join(pkg_share, "world", "ARCC_Field_2026.sdf")
    default_rviz_config_path = os.path.join(pkg_share, "rviz", "config.rviz")
    default_model_path = os.path.join(pkg_share, "urdf", "sentry.urdf.xacro")
    robot_description_config = xacro.process_file(
        default_model_path,
        mappings={"package://sentry_pkg": pkg_share}
    )
    robot_description_raw = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description_raw,
            "use_sim_time": True,
         }],
        output="screen"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        parameters=[{"use_sim_time": True}],
    )

    ros_gz_sim = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "sentry_pkg",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    bridge_config = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": bridge_config,
            "use_sim_time": True,
        }],
        output="screen",
    )
    slam_params_file = os.path.join(pkg_share, "config", "slam.yaml")    
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {"use_sim_time": True},
        ]
    )
    return LaunchDescription(
        [
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=os.path.dirname(pkg_share)
            ),
            AppendEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=os.path.dirname(pkg_share)
            ),
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
                launch_arguments={"gz_args": f"-r {world_path}", "use_sim_time": "true"}.items(),
            ),
            robot_state_publisher_node,
            rviz_node,
            ros_gz_bridge,
            ros_gz_sim,
            slam_toolbox_node
        ]
    )
