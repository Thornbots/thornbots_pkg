import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return software()


def software():
    pkg_share = get_package_share_directory("sentry_pkg")
    world_path = os.path.join(pkg_share, "world", "ARCC_Field_2026.sdf")
    default_rviz_config_path = os.path.join(pkg_share, "rviz", "config.rviz")
    default_model_path = os.path.join(pkg_share, "urdf", "sentry.urdf.xacro")
    slam_params_file = os.path.join(pkg_share, "config", "mapper_params_online_async.yaml")    
    robot_description_config = xacro.process_file(
        default_model_path,
        mappings={"package://sentry_pkg": pkg_share}
    )
    robot_description_raw = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True, "robot_description": robot_description_raw}],
        output="screen"
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": True, "robot_description": robot_description_raw}],
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
            "-name", "sentry",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        output="screen",
    )
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            slam_params_file
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
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
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
                launch_arguments={"gz_args": f"-r {world_path}"}.items(),
            ),
            joint_state_publisher_node,
            robot_state_publisher_node,
            rviz_node,
            ros_gz_bridge,
            ros_gz_sim,
            # slam_toolbox_node
        ]
    )


def hardware():
    pkg_share = get_package_share_directory("sentry_pkg")
    default_rviz_config_path = os.path.join(pkg_share, "rviz", "config.rviz")
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
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
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
    return LaunchDescription(
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
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            robot_state_publisher_node,
            joint_state_publisher_node,
            sllidar_node,
            rviz_node,
        ]
    )
