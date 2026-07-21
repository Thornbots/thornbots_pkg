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
and (outside localization_mode:=ekf) odom->root TF -- same code path for
sim and real hardware, so there's only one place pose handling can go
wrong. This package also runs its own robot_state_publisher off
sentry_pkg/urdf/sentry.urdf.xacro (fed by pose_translator's /joint_states)
rather than depending on sim's URDF/TF -- sentry_pkg owns the whole TF
tree itself now, sim only ever provides raw sensor data through the
shared real-hardware-shaped interfaces.

load_map:=true by default: deserializes map_file's saved pose graph
(map/ARCC26.posegraph + .data under sentry_pkg, saved via
slam_toolbox/srv/SerializePoseGraph) at startup and continues from it
instead of starting blank, since ARCC26 is the sentry's actual field map.
Only affects the slam/mapping localization_mode values below (slam_toolbox
always either loads or doesn't); amcl always loads map_file's .yaml
regardless (it has no concept of starting blank), and ekf mode doesn't run
any map node at all so load_map has no effect there.

localization_mode picks the whole localization scheme in one choice --
who (if anyone) owns map->odom TF, and who owns odom->root TF:
  - 'slam' (default): slam_toolbox in its own 'localization' mode owns
    map->odom, localizing root against the existing map_file rather than
    building/extending it -- the normal running mode once ARCC26 is a
    good-enough field map. pose_translator owns odom->root directly off
    /pose (passthrough). Requires load_map:=true with a real map_file --
    'slam' with load_map:=false is not a meaningful combination (there's
    no map to localize against).
  - 'mapping': slam_toolbox in its own 'mapping' mode owns map->odom,
    (re)building/extending the map instead of just localizing against it
    -- pair with load_map:=true to refine ARCC26, or load_map:=false to
    build a fresh one from scratch. This should be an occasional,
    deliberate action, not the default. use_map_saver is only turned on
    in this mode (see below) -- the map is only ever savable/updatable
    when you've deliberately opted into mapping, never as a side effect
    of ordinary localization/amcl/ekf running. pose_translator owns
    odom->root directly off /pose (passthrough), same as 'slam'.
  - 'amcl': nav2's map_server + amcl own map->odom instead, localizing
    root against map_file's saved occupancy grid (<map_file>.yaml, same
    basename slam_toolbox's posegraph uses). slam_toolbox isn't launched
    at all in this mode -- AMCL never builds a map, only localizes
    against one. map_server and amcl are nav2 lifecycle nodes, brought up
    by a lifecycle_manager node (autostart:true) rather than starting
    active on their own. pose_translator owns odom->root directly off
    /pose (passthrough), same as 'slam'/'mapping'.
  - 'ekf': no map->odom node runs at all (no slam_toolbox, no amcl/
    map_server/lifecycle_manager -- the map frame doesn't exist in this
    mode). Instead, ekf_node (robot_localization, config/ekf.yaml) fuses
    /odom (x, y, vx, vy) and /scan_odom (x, y) and owns odom->root.
    pose_translator keeps publishing /odom + /joint_states as always
    (both still needed -- /odom feeds the EKF, /joint_states feeds
    robot_state_publisher), it just stops broadcasting TF itself
    (publish_tf parameter), so exactly one of {ekf_node, pose_translator}
    ever broadcasts odom->root at a time. This mode also launches
    rf2o_laser_odometry_node (turns /scan into a second odometry-shaped
    estimate on /scan_odom, scan-matching, no TF broadcast of its own --
    publish_tf is left false since ekf_node owns odom->root, not this
    node) and head_home_scan_gate. rf2o only samples the lidar->root
    transform once, on its first received scan, and reuses that cached
    transform for its lifetime -- it assumes a rigidly-fixed sensor
    mount, which our head-mounted lidar isn't (see SESSION_NOTES.md).
    head_home_scan_gate feeds rf2o a filtered /scan_gated instead of raw
    /scan, forwarding scans only while the head is near its home (yaw ~
    0) position, so every scan rf2o ever sees (including its first,
    which fixes the cached transform) is consistent with that one head
    angle. Neither node is launched in any other localization_mode --
    nothing else reads /scan_odom or /scan_gated.
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
        description="Frame slam_toolbox/amcl/ekf_node treat as their "
                     "drift-free reference, parent of base_frame."
    )

    load_map_arg = DeclareLaunchArgument(
        "load_map", default_value="true",
        description="Deserialize map_file's saved pose graph at startup and "
                     "continue from it instead of starting blank. Defaults "
                     "on since ARCC26 (see map_file below) is the sentry's "
                     "actual field map. Only affects localization_mode:="
                     "slam/mapping; amcl always loads map_file's .yaml "
                     "regardless, and ekf runs no map node at all."
    )
    map_file_arg = DeclareLaunchArgument(
        "map_file", default_value=os.path.join(
            get_package_share_directory("sentry_pkg"), "map", "ARCC26"
        ),
        description="Path (no extension) to the map to use: slam_toolbox "
                     "reads <map_file>.posegraph/.data (see "
                     "slam_toolbox/srv/SerializePoseGraph), amcl reads "
                     "<map_file>.yaml (see nav2_map_server). Same basename, "
                     "both refer to the same saved map."
    )

    localization_mode_arg = DeclareLaunchArgument(
        "localization_mode", default_value="slam",
        choices=["slam", "mapping", "amcl", "ekf"],
        description="Selects the whole localization scheme in one choice "
                     "-- see the module docstring for what each of "
                     "slam/mapping/amcl/ekf actually launches and which "
                     "TF edges (map->odom, odom->root) it owns."
    )
    localization_mode = LaunchConfiguration("localization_mode")
    ekf_selected = PythonExpression(
        ["'", localization_mode, "' == 'ekf'"]
    )
    amcl_selected = PythonExpression(
        ["'", localization_mode, "' == 'amcl'"]
    )
    mapping_selected = PythonExpression(
        ["'", localization_mode, "' == 'mapping'"]
    )
    slam_toolbox_with_map_selected = PythonExpression(
        ["'", localization_mode, "' in ('slam', 'mapping') and '",
         LaunchConfiguration("load_map"), "' == 'true'"]
    )
    slam_toolbox_no_map_selected = PythonExpression(
        ["'", localization_mode, "' == 'mapping' and '",
         LaunchConfiguration("load_map"), "' == 'false'"]
    )
    slam_toolbox_mode_param = PythonExpression(
        ["'mapping' if '", localization_mode, "' == 'mapping' "
         "else 'localization'"]
    )
    # Map saving/updating (slam_toolbox's use_map_saver, overriding
    # config/slam.yaml's baked-in value) is only ever enabled in mapping
    # mode -- never a side effect of ordinary localization/amcl/ekf
    # running, per the module docstring.
    publish_tf_from_pose_translator = PythonExpression(
        ["'false' if '", localization_mode, "' == 'ekf' else 'true'"]
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
        condition=IfCondition(ekf_selected),
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
                     "scan (see module docstring). Only used when "
                     "localization_mode:=ekf."
    )

    # Only used by localization_mode:=ekf; nothing else reads /scan_odom
    # or /scan_gated.
    head_home_scan_gate_node = Node(
        package="sentry_pkg",
        executable="head_home_scan_gate",
        name="head_home_scan_gate",
        output="screen",
        condition=IfCondition(ekf_selected),
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
        condition=IfCondition(ekf_selected),
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
    # the key entirely rather than passing it empty. Both are also gated
    # on localization_mode being slam/mapping -- not launched at all when
    # localization_mode is amcl/ekf.
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
                "mode": slam_toolbox_mode_param,
                "use_map_saver": ParameterValue(
                    mapping_selected, value_type=bool
                ),
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
                # Always mapping: this variant only ever launches when
                # localization_mode:=mapping (see
                # slam_toolbox_no_map_selected above) -- there's no saved
                # map to localize against without load_map anyway.
                "mode": "mapping",
                "use_map_saver": True,
            },
        ],
    )

    return LaunchDescription([
        real_hardware_arg,
        lidar_serial_port_arg, lidar_baudrate_arg,
        odom_frame_arg, load_map_arg, map_file_arg, localization_mode_arg,
        home_yaw_tolerance_arg,
        dji_serial_bridge_node, lidar_node,
        pose_translator_node, ekf_node,
        head_home_scan_gate_node, scan_odom_node,
        robot_state_publisher_node,
        slam_toolbox_with_map_node, slam_toolbox_no_map_node,
        map_server_node, amcl_node, amcl_lifecycle_manager_node,
    ])
