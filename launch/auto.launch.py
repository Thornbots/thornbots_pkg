"""
Turns whatever raw /pose and /scan are on the ROS graph into a SLAM map.
The actual pose_translator/robot_state_publisher/slam_toolbox pipeline
consumes /pose + /scan only, so it's identical whether that data comes
from real hardware or sim -- no dependency on which produced them.

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

pose_translator (fed by /pose) is the sole source of /odom, /joint_states,
and odom->root TF -- same code path for sim and real hardware, so there's
only one place pose handling can go wrong. This package also runs its own
robot_state_publisher off sentry_pkg/urdf/sentry.urdf.xacro (fed by
pose_translator's /joint_states) rather than depending on sim's URDF/TF --
sentry_pkg owns the whole TF tree itself now, sim only ever provides raw
sensor data through the shared real-hardware-shaped interfaces.

load_map:=true by default: deserializes map_file's saved pose graph
(map/ARCC26.posegraph + .data under sentry_pkg, saved via
slam_toolbox/srv/SerializePoseGraph) at startup and continues mapping
from it instead of starting blank, since ARCC26 is the sentry's actual
field map. Pass load_map:=false for a fresh map (e.g. testing a
different world in sim).

rf2o_laser_odometry_node turns /scan into a second odometry-shaped estimate
on /scan_odom (scan-matching, no TF broadcast -- publish_tf is left false
since pose_translator/ekf_node owns odom->root, not this node). Not yet
consumed by anything; this is step 2 of the EKF-fusion plan in
SESSION_NOTES.md (step 3+ gates /scan_odom against wheel odometry and
fuses both in robot_localization's ekf_node before this is trusted for
odom->root).
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("sentry_pkg")
    slam_params_file = os.path.join(pkg_share, "config", "slam.yaml")
    sentry_xacro = os.path.join(pkg_share, "urdf", "sentry.urdf.xacro")

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
        description="Frame slam_toolbox treats as its drift-free reference, "
                     "parent of base_frame."
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

    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        condition=IfCondition(real_hardware),
        parameters=[{
            "serial_port": LaunchConfiguration("lidar_serial_port"),
            "serial_baudrate": ParameterValue(
                LaunchConfiguration("lidar_baudrate"), value_type=int
            ),
            "frame_id": "lidar",
            "use_sim_time": use_sim_time,
        }],
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

    scan_odom_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[{
            "laser_scan_topic": "/scan",
            "odom_topic": "/scan_odom",
            "publish_tf": False,
            "base_frame_id": "root",
            "odom_frame_id": LaunchConfiguration("odom_frame"),
            "init_pose_from_topic": "",
            "freq": 20.0,
            "use_sim_time": use_sim_time,
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
        real_hardware_arg,
        lidar_serial_port_arg, lidar_baudrate_arg,
        odom_frame_arg, load_map_arg, map_file_arg,
        dji_serial_bridge_node, lidar_node,
        pose_translator_node, scan_odom_node, robot_state_publisher_node,
        slam_toolbox_with_map_node, slam_toolbox_no_map_node,
    ])
