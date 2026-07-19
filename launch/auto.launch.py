"""
Turns whatever raw /odom and /scan are already on the ROS graph into a
SLAM map. Deliberately has no dependency on real hardware -- no lidar
driver, no serial bridge, just consumes topics -- so the same launch works
against `ros2 launch sim sim.launch.py` (which publishes raw /odom and
/scan) or a real driver later, as long as it publishes those same raw
topics. Two stages:
  1. odom_to_tf broadcasts /odom as the odom->root TF slam_toolbox needs
     (nothing upstream publishes that TF on its own). This would ideally
     be robot_localization's ekf_node, but that can't run in this
     environment right now -- see sentry_pkg/odom_to_tf.py's docstring.
  2. slam_toolbox consumes /scan + that TF to build /map.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("sentry_pkg")
    slam_params_file = os.path.join(pkg_share, "config", "slam.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Set to false when running against real hardware's /odom + /scan"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    yaw_joint_name_arg = DeclareLaunchArgument(
        "yaw_joint_name", default_value="planar_y_to_root_yaw",
        description="Sim-only workaround for gz sim's OdometryPublisher not "
                     "reporting yaw correctly for a joint-constrained base; "
                     "set to empty string on real hardware (the default "
                     "there), where pose_translator already publishes "
                     "correct orientation directly."
    )
    y_joint_name_arg = DeclareLaunchArgument(
        "y_joint_name", default_value="planar_x_to_y",
        description="Same workaround as yaw_joint_name but for /odom's Y "
                     "position, which has the same gz sim OdometryPublisher "
                     "bug; set to empty string on real hardware."
    )

    odom_to_tf_node = Node(
        package="sentry_pkg",
        executable="odom_to_tf",
        name="odom_to_tf",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "yaw_joint_name": LaunchConfiguration("yaw_joint_name"),
            "y_joint_name": LaunchConfiguration("y_joint_name"),
        }],
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg, yaw_joint_name_arg, y_joint_name_arg,
        odom_to_tf_node, slam_toolbox_node,
    ])
