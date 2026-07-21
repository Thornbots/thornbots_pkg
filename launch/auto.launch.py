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
since pose_translator/ekf_node owns odom->root, not this node). Only used
by the EKF (localization_backend:=ekf); slam_toolbox's own localization
doesn't consume /scan_odom or /scan_gated at all, so rf2o and
head_home_scan_gate are both gated off (not launched) otherwise -- no
reason to run either when nothing's reading their output.

rf2o_laser_odometry only samples the lidar->root transform once, on its
first received scan, and reuses that cached transform for its lifetime --
it assumes a rigidly-fixed sensor mount, which our head-mounted lidar
isn't (see SESSION_NOTES.md). head_home_scan_gate feeds rf2o a filtered
/scan_gated instead of raw /scan, forwarding scans only while the head is
near its home (yaw ~ 0) position, so every scan rf2o ever sees (including
its first, which fixes the cached transform) is consistent with that one
head angle.

localization_backend selects who owns odom->root TF (step 5 of the
EKF-fusion plan in SESSION_NOTES.md/ARCC_2026_SENTRY_CONTEXT.md):
  - 'passthrough' (default): pose_translator broadcasts odom->root itself,
    straight off /pose, exactly as before this EKF work -- kept as the
    default so it stays an instant, known-good fallback.
  - 'ekf': ekf_node (sentry_pkg/config/ekf.yaml) fuses /odom (x, y, vx,
    vy) and /scan_odom (x, y) and owns odom->root instead. pose_translator
    keeps publishing /odom + /joint_states either way (both still needed
    -- /odom feeds the EKF, /joint_states feeds robot_state_publisher),
    it just stops broadcasting TF itself (publish_tf parameter), so
    exactly one of {ekf_node, pose_translator} ever broadcasts odom->root
    at a time.

slam_mode picks slam_toolbox's own mode param (mapping/localization),
overriding the value baked into config/slam.yaml. Default is
'localization' -- once ARCC26 is a good-enough field map, that's the
normal running mode: slam_toolbox localizes root against the existing
map instead of continuing to build/extend it. Pass slam_mode:=mapping
(with load_map:=true to refine the existing map, or load_map:=false to
build a fresh one from scratch) when you actually want to (re)build the
map -- that should be an occasional, deliberate action, not the default.
Note localization mode requires an actual map to localize against, i.e.
load_map:=true with a real map_file; slam_mode:=localization with
load_map:=false is not a meaningful combination.

map_localizer picks who owns map->odom TF -- the map-relative correction,
same conceptual role slam_mode:=localization plays above:
  - 'slam_toolbox' (default): unchanged, the slam_toolbox_* nodes above.
  - 'amcl': nav2's map_server + amcl localize root against map_file's
    saved occupancy grid (<map_file>.yaml, same basename slam_toolbox's
    posegraph uses) instead. slam_toolbox isn't launched at all in this
    mode -- AMCL never builds a map, only localizes against one, so
    there's no mapping-mode equivalent; pair with load_map:=true and a
    real map_file, same requirement slam_mode:=localization already has.
    map_server and amcl are nav2 lifecycle nodes, brought up by a
    lifecycle_manager node (autostart:true) rather than starting active
    on their own.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("sentry_pkg")
    slam_params_file = os.path.join(pkg_share, "config", "slam.yaml")
    ekf_params_file = os.path.join(pkg_share, "config", "ekf.yaml")
    amcl_params_file = os.path.join(pkg_share, "config", "amcl.yaml")
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

    localization_backend_arg = DeclareLaunchArgument(
        "localization_backend", default_value="passthrough",
        choices=["ekf", "passthrough"],
        description="Who owns odom->root TF. 'passthrough' (default): "
                     "pose_translator broadcasts it directly off /pose, "
                     "unchanged from before the EKF work -- kept as the "
                     "default so it's an instant fallback. 'ekf': "
                     "ekf_node (config/ekf.yaml) fuses /odom + /scan_odom "
                     "and owns it instead; pose_translator's own TF "
                     "broadcast is disabled in that mode (see module "
                     "docstring)."
    )
    localization_backend = LaunchConfiguration("localization_backend")
    publish_tf_from_pose_translator = PythonExpression(
        ["'true' if '", localization_backend, "' == 'passthrough' else 'false'"]
    )
    ekf_backend_selected = PythonExpression(
        ["'", localization_backend, "' == 'ekf'"]
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

    slam_mode_arg = DeclareLaunchArgument(
        "slam_mode", default_value="localization",
        choices=["mapping", "localization"],
        description="slam_toolbox's own mode param, overriding config/"
                     "slam.yaml's baked-in value. 'localization' (default): "
                     "localize root against the existing map_file rather "
                     "than continuing to build/extend it -- the normal "
                     "running mode once ARCC26 is a good-enough field map. "
                     "'mapping': (re)build the map -- pair with load_map:="
                     "true to refine ARCC26, or load_map:=false to start "
                     "fresh. slam_mode:=localization requires load_map:="
                     "true with a real map_file; there's no map to "
                     "localize against otherwise."
    )

    map_localizer_arg = DeclareLaunchArgument(
        "map_localizer", default_value="slam_toolbox",
        choices=["slam_toolbox", "amcl"],
        description="Who owns map->odom TF. 'slam_toolbox' (default): "
                     "unchanged, the slam_toolbox_* nodes below, mode "
                     "picked by slam_mode above. 'amcl': nav2's "
                     "map_server + amcl (config/amcl.yaml) localize "
                     "root against map_file's saved occupancy grid "
                     "(<map_file>.yaml) instead; slam_toolbox isn't "
                     "launched at all in this mode -- AMCL never builds "
                     "a map, only localizes against one, so pair with "
                     "load_map:=true and a real map_file (see module "
                     "docstring)."
    )
    map_localizer = LaunchConfiguration("map_localizer")
    amcl_selected = PythonExpression(
        ["'", map_localizer, "' == 'amcl'"]
    )
    slam_toolbox_with_map_selected = PythonExpression(
        ["'", map_localizer, "' == 'slam_toolbox' and '",
         LaunchConfiguration("load_map"), "' == 'true'"]
    )
    slam_toolbox_no_map_selected = PythonExpression(
        ["'", map_localizer, "' == 'slam_toolbox' and '",
         LaunchConfiguration("load_map"), "' == 'false'"]
    )
    map_yaml_file = PythonExpression(
        ["'", LaunchConfiguration("map_file"), "' + '.yaml'"]
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
            "publish_tf": ParameterValue(
                publish_tf_from_pose_translator, value_type=bool
            ),
        }],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=IfCondition(ekf_backend_selected),
        parameters=[
            ekf_params_file,
            {
                "use_sim_time": use_sim_time,
                "odom_frame": LaunchConfiguration("odom_frame"),
                "base_link_frame": "root",
                # Must match odom_frame, not base_link_frame -- see
                # config/ekf.yaml's comment on world_frame.
                "world_frame": LaunchConfiguration("odom_frame"),
            },
        ],
    )

    home_yaw_tolerance_arg = DeclareLaunchArgument(
        "home_yaw_tolerance", default_value="0.05",
        description="Max |head_yaw| (radians) for head_home_scan_gate to "
                     "treat the head as 'home' and forward scans to rf2o. "
                     "Keeps every scan rf2o ever sees consistent with the "
                     "single lidar->root transform it caches on its first "
                     "scan (see module docstring)."
    )

    # Only feed the EKF (localization_backend:=ekf); slam_toolbox's own
    # localization doesn't consume /scan_gated or /scan_odom at all, so
    # there's no reason to run rf2o/the gate otherwise.
    head_home_scan_gate_node = Node(
        package="sentry_pkg",
        executable="head_home_scan_gate",
        name="head_home_scan_gate",
        output="screen",
        condition=IfCondition(ekf_backend_selected),
        parameters=[{
            "use_sim_time": use_sim_time,
            "home_yaw_tolerance": ParameterValue(
                LaunchConfiguration("home_yaw_tolerance"), value_type=float
            ),
        }],
    )

    scan_odom_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        condition=IfCondition(ekf_backend_selected),
        parameters=[{
            "laser_scan_topic": "/scan_gated",
            "odom_topic": "/scan_odom",
            "publish_tf": False,
            "base_frame_id": "root",
            "odom_frame_id": LaunchConfiguration("odom_frame"),
            "init_pose_from_topic": "",
            "freq": 20.0,
            "use_sim_time": use_sim_time,
        }],
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        condition=IfCondition(amcl_selected),
        parameters=[{
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml_file,
        }],
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        condition=IfCondition(amcl_selected),
        parameters=[
            amcl_params_file,
            {
                "use_sim_time": use_sim_time,
                "odom_frame_id": LaunchConfiguration("odom_frame"),
                "base_frame_id": "root",
                "global_frame_id": "map",
                "scan_topic": "/scan",
            },
        ],
    )

    # map_server/amcl are nav2 lifecycle nodes -- they start unconfigured
    # and inactive on their own; this brings both up automatically
    # instead of requiring a manual configure/activate service call.
    amcl_lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        condition=IfCondition(amcl_selected),
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
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
    # the key entirely rather than passing it empty. Both are also
    # gated on map_localizer:=slam_toolbox -- not launched at all when
    # map_localizer:=amcl owns map->odom instead.
    slam_toolbox_with_map_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        condition=IfCondition(slam_toolbox_with_map_selected),
        parameters=[
            slam_params_file,
            {
                "use_sim_time": use_sim_time,
                "odom_frame": LaunchConfiguration("odom_frame"),
                "map_file_name": LaunchConfiguration("map_file"),
                "map_start_pose": [0.0, 0.0, 0.0],
                "mode": LaunchConfiguration("slam_mode"),
            },
        ],
    )
    slam_toolbox_no_map_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        condition=IfCondition(slam_toolbox_no_map_selected),
        parameters=[
            slam_params_file,
            {
                "use_sim_time": use_sim_time,
                "odom_frame": LaunchConfiguration("odom_frame"),
                # Always mapping: there's no saved map to localize
                # against without load_map, regardless of slam_mode.
                "mode": "mapping",
            },
        ],
    )

    return LaunchDescription([
        real_hardware_arg,
        lidar_serial_port_arg, lidar_baudrate_arg,
        odom_frame_arg, localization_backend_arg,
        load_map_arg, map_file_arg, slam_mode_arg, map_localizer_arg,
        home_yaw_tolerance_arg,
        dji_serial_bridge_node, lidar_node,
        pose_translator_node, ekf_node,
        head_home_scan_gate_node, scan_odom_node,
        robot_state_publisher_node,
        slam_toolbox_with_map_node, slam_toolbox_no_map_node,
        map_server_node, amcl_node, amcl_lifecycle_manager_node,
    ])
