"""
Gets /pose and /scan onto the ROS graph and owns the robot description,
then hands off to sentry_localization for the actual odom->root/map->odom
localization work.

real_hardware:=true by default: also launches dji_serial_bridge_node
(publishes /pose from the Type-C board's serial link) and sllidar_ros2's
driver (publishes /scan from the RPLIDAR A2M8 over its own serial link,
serial port/baud set via lidar_serial_port/lidar_baudrate). This is now
the default because running against real hardware is the common case.
real_hardware also drives use_sim_time (there's no separate arg for it):
false/wall-clock when real_hardware is true, true when it's false, since
that's exactly when /clock exists to use instead.

When running against sim instead (`ros2 launch sim sim.launch.py`, which
runs sim/pose_emulator.py to publish /pose in the same
dji_serial_bridge/msg/RobotPose format real hardware sends, plus raw
/scan via its own gz bridge), launch this with real_hardware:=false so it
doesn't also try to open the real serial devices, and so it uses sim's
/clock.

pose_translator (fed by /pose) turns raw hardware pose into /odom (raw,
uncorrected wheel odometry) and /joint_states -- same code path for sim
and real hardware, so there's only one place pose handling can go wrong.
This package also runs its own robot_state_publisher off
sentry_pkg/urdf/sentry.urdf.xacro (fed by pose_translator's /joint_states)
rather than depending on sim's URDF/TF -- sentry_pkg owns the whole TF
tree itself, sim only ever provides raw sensor data through the shared
real-hardware-shaped interfaces.

sentry_pkg no longer computes odom->root itself: /odom + /scan are handed
to sentry_localization (included below), which always publishes the
localized result on /localization/odom regardless of which backend
localization_mode selects. odom_tf_broadcaster subscribes that topic and
broadcasts the actual odom->root TF -- so this package never needs to
know which localization_mode is active. See sentry_pkg/README.md and
sentry_localization/README.md for the full pipeline and the meaning of
each localization_mode/load_map/map_file arg forwarded below.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("sentry_pkg")
    sentry_xacro = os.path.join(pkg_share, "urdf", "sentry.urdf.xacro")
    localization_launch = os.path.join(
        get_package_share_directory("sentry_localization"),
        "launch", "localization.launch.py",
    )

    real_hardware_arg = DeclareLaunchArgument(
        "real_hardware", default_value="true",
        description="Launch dji_serial_bridge_node (the Type-C board's /pose "
                     "source) and sllidar_ros2's driver (/scan) directly. "
                     "True by default since running against real hardware is "
                     "now the default; set false when running against sim, "
                     "which provides /pose (via pose_emulator) and /scan "
                     "itself. Also drives use_sim_time (false when "
                     "real_hardware is true -- wall clock -- true otherwise, "
                     "since sim publishes /clock): no separate use_sim_time "
                     "arg, they're the same knob."
    )
    real_hardware = LaunchConfiguration("real_hardware")
    use_sim_time = PythonExpression(
        ["'false' if '", real_hardware, "' == 'true' else 'true'"]
    )

    lidar_serial_port_arg = DeclareLaunchArgument(
        "lidar_serial_port", default_value="/dev/ttyUSB0",
        description="Serial device for the RPLIDAR A2M8, only used when "
                     "real_hardware:=true."
    )
    lidar_baudrate_arg = DeclareLaunchArgument(
        "lidar_baudrate", default_value="115200",
        description="Baud rate for the RPLIDAR A2M8, only used when "
                     "real_hardware:=true."
    )

    odom_frame_arg = DeclareLaunchArgument(
        "odom_frame", default_value="odom",
        description="Frame sentry_localization/odom_tf_broadcaster treat as "
                     "their drift-free reference, parent of base_frame."
    )

    load_map_arg = DeclareLaunchArgument(
        "load_map", default_value="true",
        description="Forwarded to sentry_localization -- see "
                     "sentry_localization's localization.launch.py for what "
                     "this controls."
    )
    map_file_arg = DeclareLaunchArgument(
        "map_file", default_value=os.path.join(
            get_package_share_directory("sentry_localization"),
            "map", "clean_map"
        ),
        description="Forwarded to sentry_localization -- path (no "
                     "extension) to the map to use. Default is clean_map; "
                     "pass map_file:=<sentry_localization share>/map/ARCC26 "
                     "explicitly for localization_mode:=slam/mapping until "
                     "clean_map has a real posegraph (see "
                     "sentry_localization's localization.launch.py)."
    )

    localization_mode_arg = DeclareLaunchArgument(
        "localization_mode", default_value="slam",
        choices=["slam", "mapping", "amcl", "ekf"],
        description="Forwarded to sentry_localization -- selects the whole "
                     "localization scheme. See sentry_localization's "
                     "localization.launch.py module docstring for what each "
                     "value launches."
    )

    home_yaw_tolerance_arg = DeclareLaunchArgument(
        "home_yaw_tolerance", default_value="0.05",
        description="Forwarded to sentry_localization -- only used when "
                     "localization_mode:=ekf. See "
                     "sentry_localization/head_home_scan_gate.py."
    )

    # device/baudrate for dji_serial_bridge_node are left at its own defaults
    # (/dev/ttyTHS1, 115200) -- only the lidar's serial settings are exposed
    # as launch args here.
    dji_serial_bridge_node = Node(
        package="dji_serial_bridge",
        executable="dji_serial_bridge_node",
        name="dji_serial_bridge",
        output="screen",
        condition=IfCondition(real_hardware),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Published as scan_raw, not scan -- lidar_self_filter_node below is the
    # only thing that publishes the final /scan, for both real_hardware and
    # sim (see that node's own docstring for why this needs to be a software
    # filter rather than something modeled in the URDF/world).
    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        condition=IfCondition(real_hardware),
        remappings=[("scan", "scan_raw")],
        parameters=[{
            "serial_port": LaunchConfiguration("lidar_serial_port"),
            "serial_baudrate": ParameterValue(
                LaunchConfiguration("lidar_baudrate"), value_type=int
            ),
            "frame_id": "lidar",
            "use_sim_time": use_sim_time,
        }],
    )

    lidar_self_filter_node = Node(
        package="sentry_pkg",
        executable="lidar_self_filter",
        name="lidar_self_filter",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    pose_translator_node = Node(
        package="sentry_pkg",
        executable="pose_translator",
        name="pose_translator",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "odom_frame": LaunchConfiguration("odom_frame"),
        }],
    )

    odom_tf_broadcaster_node = Node(
        package="sentry_pkg",
        executable="odom_tf_broadcaster",
        name="odom_tf_broadcaster",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "odom_frame": LaunchConfiguration("odom_frame"),
        }],
    )

    robot_description = ParameterValue(
        Command(["xacro ", sentry_xacro]), value_type=str
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )

    localization_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "odom_frame": LaunchConfiguration("odom_frame"),
            "load_map": LaunchConfiguration("load_map"),
            "map_file": LaunchConfiguration("map_file"),
            "localization_mode": LaunchConfiguration("localization_mode"),
            "home_yaw_tolerance": LaunchConfiguration("home_yaw_tolerance"),
        }.items(),
    )

    return LaunchDescription([
        real_hardware_arg,
        lidar_serial_port_arg, lidar_baudrate_arg,
        odom_frame_arg, load_map_arg, map_file_arg, localization_mode_arg,
        home_yaw_tolerance_arg,
        dji_serial_bridge_node, lidar_node, lidar_self_filter_node,
        pose_translator_node, odom_tf_broadcaster_node,
        robot_state_publisher_node,
        localization_launch_include,
    ])
