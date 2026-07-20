"""
Turns whatever raw /odom and /scan are already on the ROS graph into a
SLAM map. Deliberately has no dependency on real hardware -- no lidar
driver, no serial bridge, just consumes topics -- so the same launch works
against `ros2 launch sim sim.launch.py` (which publishes raw /odom and
/scan) or a real driver later, as long as it publishes those same raw
topics.

publish_odom_tf:=true, odom_frame:=odom by default (both sim and real
hardware). sim/urdf/sentry.urdf.xacro's "root" link is a genuinely free
6DOF body with no parent joint at all (see that file), so
robot_state_publisher has nothing to publish odom->root from on its own --
odom_to_tf, fed by OdometryPublisher's /odom, is the only source, same as
real hardware. (An earlier version of the sim URDF drove root through a
world->planar_x->planar_y->yaw_base->root joint chain instead, which let
robot_state_publisher publish that transform directly -- but that also
meant *two* different nodes were broadcasting a parent for "root"
whenever odom_to_tf ran alongside it, an invalid TF tree that caused
"message filter queue full" spam in slam_toolbox. That whole joint chain
is gone now, so there's no longer a second broadcaster to conflict with.)

load_map:=true by default: deserializes map_file's saved pose graph
(map/ARCC26.posegraph + .data under sentry_pkg, saved via
slam_toolbox/srv/SerializePoseGraph) at startup and continues mapping
from it instead of starting blank, since ARCC26 is the sentry's actual
field map. Pass load_map:=false for a fresh map (e.g. testing a
different world in sim).
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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

    publish_odom_tf_arg = DeclareLaunchArgument(
        "publish_odom_tf", default_value="true",
        description="Run odom_to_tf to broadcast odom->root from /odom. "
                     "True for both sim and real hardware now; nothing else "
                     "publishes that transform on its own."
    )
    odom_frame_arg = DeclareLaunchArgument(
        "odom_frame", default_value="odom",
        description="Frame slam_toolbox treats as its drift-free reference, "
                     "parent of base_frame."
    )

    yaw_joint_name_arg = DeclareLaunchArgument(
        "yaw_joint_name", default_value="",
        description="Only needed if /odom's orientation field is wrong for "
                     "some future base design (see odom_to_tf.py's "
                     "docstring); root never rotates on the current sim "
                     "base, so /odom's orientation is trusted directly."
    )
    y_joint_name_arg = DeclareLaunchArgument(
        "y_joint_name", default_value="",
        description="Only needed if /odom's Y field is wrong for some "
                     "future base design (see odom_to_tf.py's docstring); "
                     "empty by default, trusting /odom's Y directly."
    )

    load_map_arg = DeclareLaunchArgument(
        "load_map", default_value="true",
        description="Deserialize map_file's saved pose graph at startup and "
                     "continue mapping from it instead of starting blank. "
                     "Defaults on since ARCC26 (see map_file below) is the "
                     "sentry's actual field map."
    )
    map_file_arg = DeclareLaunchArgument(
        "map_file", default_value=os.path.join(
            get_package_share_directory("sentry_pkg"), "map", "ARCC26"
        ),
        description="Path (no extension) to a slam_toolbox-serialized pose "
                     "graph (<map_file>.posegraph/.data, see "
                     "slam_toolbox/srv/SerializePoseGraph) to resume "
                     "mapping from. Only used when load_map:=true."
    )

    odom_to_tf_node = Node(
        package="sentry_pkg",
        executable="odom_to_tf",
        name="odom_to_tf",
        output="screen",
        condition=IfCondition(LaunchConfiguration("publish_odom_tf")),
        parameters=[{
            "use_sim_time": use_sim_time,
            "yaw_joint_name": LaunchConfiguration("yaw_joint_name"),
            "y_joint_name": LaunchConfiguration("y_joint_name"),
        }],
    )

    # Two variants of the same node (mirrors sim.launch.py's gz_sim/
    # gz_sim_headless split): map_file_name is only meaningful to
    # slam_toolbox when actually set, and launch Node parameter dicts are
    # static, so load_map:=false needs a version of this node that omits
    # the key entirely rather than passing it empty.
    slam_toolbox_with_map_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        condition=IfCondition(LaunchConfiguration("load_map")),
        parameters=[
            slam_params_file,
            {
                "use_sim_time": use_sim_time,
                "odom_frame": LaunchConfiguration("odom_frame"),
                "map_file_name": LaunchConfiguration("map_file"),
                "map_start_pose": [0.0, 0.0, 0.0],
            },
        ],
    )
    slam_toolbox_no_map_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("load_map")),
        parameters=[
            slam_params_file,
            {
                "use_sim_time": use_sim_time,
                "odom_frame": LaunchConfiguration("odom_frame"),
            },
        ],
    )

    return LaunchDescription([
        use_sim_time_arg, publish_odom_tf_arg, odom_frame_arg,
        yaw_joint_name_arg, y_joint_name_arg,
        load_map_arg, map_file_arg,
        odom_to_tf_node, slam_toolbox_with_map_node, slam_toolbox_no_map_node,
    ])
