# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Launch thornbots_pkg's core nodes and hand off to sentry_localization.

Gets /pose and /scan onto the graph, owns the robot description, then
hands off to sentry_localization for odom->root/map->odom localization.
See README.md for design rationale and per-node breakdown.

real_hardware:=true (default) launches dji_serial_bridge_node + sllidar_ros2
and sets use_sim_time accordingly; run real_hardware:=false against
sim.launch.py instead.
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
    pkg_share = get_package_share_directory('thornbots_pkg')
    sentry_xacro = os.path.join(pkg_share, 'urdf', 'sentry.urdf.xacro')
    localization_launch = os.path.join(
        get_package_share_directory('sentry_localization'),
        'launch', 'localization.launch.py',
    )

    real_hardware_arg = DeclareLaunchArgument(
        'real_hardware', default_value='true',
        description="Launch dji_serial_bridge_node (the Type-C board's /pose "
        "source) and sllidar_ros2's driver (/scan) directly. "
        'True by default since running against real hardware is '
        'now the default; set false when running against sim, '
        'which provides /pose (via pose_emulator) and /scan '
        'itself. Also drives use_sim_time (false when '
        'real_hardware is true -- wall clock -- true otherwise, '
        'since sim publishes /clock): no separate use_sim_time '
        "arg, they're the same knob."
    )
    real_hardware = LaunchConfiguration('real_hardware')
    use_sim_time = PythonExpression(
        ["'false' if '", real_hardware, "' == 'true' else 'true'"]
    )

    lidar_serial_port_arg = DeclareLaunchArgument(
        'lidar_serial_port', default_value='/dev/ttyUSB0',
        description='Serial device for the RPLIDAR A2M8, only used when '
        'real_hardware:=true.'
    )
    lidar_baudrate_arg = DeclareLaunchArgument(
        'lidar_baudrate', default_value='115200',
        description='Baud rate for the RPLIDAR A2M8, only used when '
        'real_hardware:=true.'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame', default_value='odom',
        description='Frame sentry_localization/odom_tf_broadcaster treat as '
        'their drift-free reference, parent of base_frame.'
    )

    load_map_arg = DeclareLaunchArgument(
        'load_map', default_value='true',
        description='Forwarded to sentry_localization -- see '
        "sentry_localization's localization.launch.py for what "
        'this controls.'
    )
    map_file_arg = DeclareLaunchArgument(
        'map_file', default_value=os.path.join(
            get_package_share_directory('sentry_localization'),
            'map', 'clean_map'
        ),
        description='Forwarded to sentry_localization -- path (no '
        'extension) to the map to use. Default is clean_map; '
        'pass map_file:=<sentry_localization share>/map/ARCC26 '
        'explicitly for localization_mode:=slam/mapping until '
        'clean_map has a real posegraph (see '
        "sentry_localization's localization.launch.py)."
    )

    localization_mode_arg = DeclareLaunchArgument(
        'localization_mode', default_value='amcl',
        choices=['slam', 'mapping', 'amcl', 'none'],
        description='Forwarded to sentry_localization -- selects who owns '
        "map->odom. See sentry_localization's "
        'localization.launch.py module docstring for what each '
        'value launches.'
    )
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf', default_value='false',
        description='Forwarded to sentry_localization -- whether odom->root '
        'is EKF-fused instead of passed through raw from /odom. '
        'Independent of localization_mode. See '
        "sentry_localization's localization.launch.py module "
        'docstring.'
    )

    enable_cv_target_bridge_arg = DeclareLaunchArgument(
        'enable_cv_target_bridge', default_value='true',
        description="Launch point_to_cv_target to turn target_tracker's "
        '/cv/target_state into the root-frame /cv/target aim '
        'point (plus /cv/panel_polygon off panel_topic). '
        'Independent of real_hardware -- consumed by mcb_relay '
        "when real_hardware:=true, and by sim's cv_head_aim "
        'node when running against sim.'
    )
    panel_topic_arg = DeclareLaunchArgument(
        'panel_topic', default_value='/cv/panel_detection',
        description='Singular PanelDetection topic (the picked panel) -- '
        'published by target_selector, consumed by '
        'point_to_cv_target.'
    )
    lead_enabled_arg = DeclareLaunchArgument(
        'lead_enabled', default_value='false',
        description='point_to_cv_target: apply the intercept/lead solve '
        'to /cv/target. false emits the raw '
        'target_tracker centre with no prediction -- one '
        'param flip between before/after.'
    )
    firmware_latency_s_arg = DeclareLaunchArgument(
        'firmware_latency_s', default_value='0.0',
        description='point_to_cv_target: fixed MCB/UART latency added to '
        'the measured now-detection_stamp latency for the '
        "intercept solve's tau. Needs measuring on hardware; "
        '0.0 is a placeholder.'
    )
    v_muzzle_arg = DeclareLaunchArgument(
        'v_muzzle', default_value='25.0',
        description='point_to_cv_target: projectile speed (m/s) used only '
        "to size the intercept solve's prediction horizon -- "
        'Type-C computes the real ballistic flight time.'
    )
    cv_target_publish_rate_hz_arg = DeclareLaunchArgument(
        'cv_target_publish_rate_hz', default_value='30.0',
        description='point_to_cv_target: /cv/target publish rate, '
        'decoupled from the ~60Hz detection rate.'
    )

    enable_target_selector_arg = DeclareLaunchArgument(
        'enable_target_selector', default_value='true',
        description="Launch target_selector to turn roi_depth_query's "
        '/cv/panel_detections (PanelDetectionArray, ALL '
        'detections) into the singular panel_topic. Independent '
        'of real_hardware, same as enable_cv_target_bridge.'
    )
    panel_array_topic_arg = DeclareLaunchArgument(
        'panel_array_topic', default_value='/cv/panel_detections',
        description='PanelDetectionArray topic published by '
        'roi_depth_query/roi_depth_node, consumed by '
        'target_selector.'
    )
    ref_sys_topic_arg = DeclareLaunchArgument(
        'ref_sys_topic', default_value='/dji_serial_bridge/ref_sys',
        description='RefSysStatus topic target_selector reads for the '
        'referee team colour (allied-detection filtering).'
    )
    center_weight_arg = DeclareLaunchArgument(
        'center_weight', default_value='1.0',
        description='target_selector: weight of 3D centrality in the panel '
        'score.'
    )
    priority_class_bonus_arg = DeclareLaunchArgument(
        'priority_class_bonus', default_value='0.5',
        description='target_selector: score bonus for priority_class_ids.'
    )
    priority_class_ids_arg = DeclareLaunchArgument(
        'priority_class_ids', default_value='[2, 6]',
        description='target_selector: class IDs treated as high-value '
        'targets.'
    )

    enable_target_tracker_arg = DeclareLaunchArgument(
        'enable_target_tracker', default_value='true',
        description='Launch target_tracker to publish /cv/target_state '
        '(spin-centre KF estimate in odom) from panel_topic. '
        'Independent of real_hardware, same as '
        'enable_target_selector.'
    )
    target_state_topic_arg = DeclareLaunchArgument(
        'target_state_topic', default_value='/cv/target_state',
        description='TargetState topic published by target_tracker, '
        "consumed by point_to_cv_target's intercept solver."
    )

    # device/baudrate for dji_serial_bridge_node are left at its own defaults
    # (/dev/ttyTHS1, 115200) -- only the lidar's serial settings are exposed
    # as launch args here.
    dji_serial_bridge_node = Node(
        package='dji_serial_bridge',
        executable='dji_serial_bridge_node',
        name='dji_serial_bridge',
        output='screen',
        condition=IfCondition(real_hardware),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Sole relay onto dji_serial_bridge_node's topics -- sentry_localization
    # (relocalize corrections) and the CV pipeline (targets) publish on this
    # node's input topics instead of touching dji_serial_bridge_node
    # directly; see thornbots_pkg/mcb_relay.py's docstring. Only meaningful
    # alongside dji_serial_bridge_node itself, hence the same real_hardware
    # gate.
    mcb_relay_node = Node(
        package='thornbots_pkg',
        executable='mcb_relay',
        name='mcb_relay',
        output='screen',
        condition=IfCondition(real_hardware),
    )

    # Turns target_tracker's /cv/target_state into the root-frame CVTarget
    # published on /cv/target (and panel_topic's corners into
    # /cv/panel_polygon) -- consumed by mcb_relay (real_hardware:=true)
    # and/or sim's cv_head_aim node (real_hardware:=false), so this runs in
    # both modes; enable_cv_target_bridge lets you disable it if you intend
    # to publish /cv/target yourself.
    point_to_cv_target_node = Node(
        package='thornbots_pkg',
        executable='point_to_cv_target',
        name='point_to_cv_target',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_cv_target_bridge')),
        parameters=[{
            'use_sim_time': use_sim_time,
            'panel_topic': LaunchConfiguration('panel_topic'),
            'target_state_topic': LaunchConfiguration('target_state_topic'),
            'output_topic': '/cv/target',
            'odom_frame': LaunchConfiguration('odom_frame'),
            'lead_enabled': ParameterValue(
                LaunchConfiguration('lead_enabled'), value_type=bool
            ),
            'firmware_latency_s': ParameterValue(
                LaunchConfiguration('firmware_latency_s'), value_type=float
            ),
            'v_muzzle': ParameterValue(
                LaunchConfiguration('v_muzzle'), value_type=float
            ),
            'cv_target_publish_rate_hz': ParameterValue(
                LaunchConfiguration('cv_target_publish_rate_hz'), value_type=float
            ),
        }],
    )

    # Picks the winning panel out of roi_depth_query's /cv/panel_detections
    # (ALL detections) and republishes it as the singular panel_topic --
    # upstream of point_to_cv_target_node in the pipeline, hence its own
    # enable_target_selector toggle (see run_shot_hit_tests.py's robot_tf
    # launch for why these two toggles are independent of
    # enable_cv_target_bridge: it needs point_to_cv_target's/target_selector's
    # own standalone copies without auto.launch.py launching a second one).
    target_selector_node = Node(
        package='thornbots_pkg',
        executable='target_selector',
        name='target_selector',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_target_selector')),
        parameters=[{
            'panel_array_topic':    LaunchConfiguration('panel_array_topic'),
            'panel_topic':          LaunchConfiguration('panel_topic'),
            'ref_sys_topic':        LaunchConfiguration('ref_sys_topic'),
            'center_weight':        LaunchConfiguration('center_weight'),
            'priority_class_bonus': LaunchConfiguration('priority_class_bonus'),
            'priority_class_ids':   LaunchConfiguration('priority_class_ids'),
        }],
    )

    # Estimates the tracked robot's spin-centre in odom from panel_topic --
    # downstream of target_selector, upstream of point_to_cv_target's
    # intercept solve. Own enable toggle for the same reason as
    # target_selector: run_shot_hit_tests.py's robot_tf launch needs its own
    # standalone copy without auto.launch.py launching a second one.
    target_tracker_node = Node(
        package='thornbots_pkg',
        executable='target_tracker',
        name='target_tracker',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_target_tracker')),
        parameters=[{
            'use_sim_time': use_sim_time,
            'panel_topic': LaunchConfiguration('panel_topic'),
            'output_topic': LaunchConfiguration('target_state_topic'),
            'odom_frame': LaunchConfiguration('odom_frame'),
        }],
    )

    # Published as scan_raw, not scan -- lidar_self_filter_node below is the
    # only thing that publishes the final /scan, for both real_hardware and
    # sim (see that node's own docstring for why this needs to be a software
    # filter rather than something modeled in the URDF/world).
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        condition=IfCondition(real_hardware),
        remappings=[('scan', 'scan_raw')],
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_serial_port'),
            'serial_baudrate': ParameterValue(
                LaunchConfiguration('lidar_baudrate'), value_type=int
            ),
            'frame_id': 'lidar',
            'use_sim_time': use_sim_time,
        }],
    )

    lidar_self_filter_node = Node(
        package='thornbots_pkg',
        executable='lidar_self_filter',
        name='lidar_self_filter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # FASTRTPS_DEFAULT_PROFILES_FILE forces UDP-only transport (no shared
    # memory) for just these two nodes: they've been observed hanging in
    # rcl_node_init/FastDDS SharedMemTransport::CreateInputChannelResource
    # on startup, before rclpy.spin() even runs, once /dev/shm accumulates
    # many stale fastrtps_* segments from earlier SIGKILLed runs -- SIGINT
    # and SIGTERM are never handled because the hang is below the Python
    # signal-check point. See config/fastdds_no_shm.xml.
    no_shm_env = {
        'FASTRTPS_DEFAULT_PROFILES_FILE': os.path.join(
            pkg_share, 'config', 'fastdds_no_shm.xml'
        )
    }

    pose_translator_node = Node(
        package='thornbots_pkg',
        executable='pose_translator',
        name='pose_translator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': LaunchConfiguration('odom_frame'),
        }],
        additional_env=no_shm_env,
    )

    odom_tf_broadcaster_node = Node(
        package='thornbots_pkg',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': LaunchConfiguration('odom_frame'),
        }],
        additional_env=no_shm_env,
    )

    robot_description = ParameterValue(
        Command(['xacro ', sentry_xacro]), value_type=str
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    localization_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'odom_frame': LaunchConfiguration('odom_frame'),
            'load_map': LaunchConfiguration('load_map'),
            'map_file': LaunchConfiguration('map_file'),
            'localization_mode': LaunchConfiguration('localization_mode'),
            'use_ekf': LaunchConfiguration('use_ekf'),
        }.items(),
    )

    return LaunchDescription([
        real_hardware_arg,
        lidar_serial_port_arg, lidar_baudrate_arg,
        odom_frame_arg, load_map_arg, map_file_arg, localization_mode_arg,
        use_ekf_arg,
        enable_cv_target_bridge_arg, panel_topic_arg,
        lead_enabled_arg, firmware_latency_s_arg, v_muzzle_arg,
        cv_target_publish_rate_hz_arg,
        enable_target_selector_arg, panel_array_topic_arg, ref_sys_topic_arg,
        center_weight_arg, priority_class_bonus_arg, priority_class_ids_arg,
        enable_target_tracker_arg, target_state_topic_arg,
        dji_serial_bridge_node, mcb_relay_node,
        target_selector_node, target_tracker_node, point_to_cv_target_node,
        lidar_node, lidar_self_filter_node,
        pose_translator_node, odom_tf_broadcaster_node,
        robot_state_publisher_node,
        localization_launch_include,
    ])
