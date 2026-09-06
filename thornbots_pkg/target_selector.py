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
Pick the single panel to shoot at from all detections.

target_selector.py — WHAT to shoot: picks one panel per frame out of
roi_depth_node's /cv/panel_detections (PanelDetectionArray, 3D, all
detections). Replaces detection_picker_node.cpp, moved downstream of
depth so robot grouping works in metres, not pixels. See README.md's
### target_selector.py Notes for the clustering-rule tradeoff and scoring
history.

Pure scoring/grouping/hysteresis logic lives in target_selector_core.py
(no rclpy import) so it's unit-testable standalone -- see
test/test_target_selector.py. This module wires ROS I/O around it.

Pipeline: /cv/panel_detections -> team filter -> per-frame score/pick
per candidate robot cluster -> robot-level hysteresis -> best panel of
the winning cluster republished on /cv/panel_detection (PanelDetection,
singular) -- unchanged shape/topic so point_to_cv_target.py needs no
changes.
"""
import math

from dji_serial_bridge.msg import PanelDetection, PanelDetectionArray, RefSysStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from thornbots_pkg.target_selector_core import (
    centrality_3d, cluster_centroid, compute_score, eligible,
    group_panels, RobotHysteresis,
)


class TargetSelector(Node):

    def __init__(self):
        super().__init__('target_selector')

        self.declare_parameter('panel_array_topic', '/cv/panel_detections')
        self.declare_parameter('panel_topic', '/cv/panel_detection')
        self.declare_parameter('ref_sys_topic', '/dji_serial_bridge/ref_sys')
        self.declare_parameter('min_score', 0.0)
        self.declare_parameter('center_weight', 1.0)
        self.declare_parameter('priority_class_bonus', 0.5)
        self.declare_parameter('priority_class_ids', [2, 6])
        self.declare_parameter('centrality_max_angle_rad', math.radians(45.0))
        self.declare_parameter('panel_group_radius_m', 0.4)
        self.declare_parameter('switch_margin', 0.3)
        self.declare_parameter('switch_hold_frames', 5)

        gp = self.get_parameter
        panel_array_topic = gp('panel_array_topic').value
        panel_topic = gp('panel_topic').value
        self.ref_sys_topic = gp('ref_sys_topic').value
        self.min_score = float(gp('min_score').value)
        self.center_weight = float(gp('center_weight').value)
        self.priority_class_bonus = float(gp('priority_class_bonus').value)
        self.priority_class_ids = {int(c) for c in gp('priority_class_ids').value}
        self.centrality_max_angle_rad = float(gp('centrality_max_angle_rad').value)
        self.panel_group_radius_m = float(gp('panel_group_radius_m').value)

        self.hysteresis = RobotHysteresis(
            switch_margin=float(gp('switch_margin').value),
            switch_hold_frames=int(gp('switch_hold_frames').value))

        self.is_blue_team = None  # None until first RefSysStatus

        self.pub = self.create_publisher(PanelDetection, panel_topic, 10)
        self.array_sub = self.create_subscription(
            PanelDetectionArray, panel_array_topic, self.on_array, 10)
        # Matches dji_serial_bridge_node's ~/ref_sys SensorDataQoS publisher.
        self.ref_sys_sub = self.create_subscription(
            RefSysStatus, self.ref_sys_topic, self.on_ref_sys, qos_profile_sensor_data)

        self.get_logger().info(
            f'target_selector ready\n'
            f'  {panel_array_topic} -> {panel_topic}\n'
            f'  score = conf + {self.center_weight}*centrality + '
            f'{self.priority_class_bonus} if class in {sorted(self.priority_class_ids)}'
            f'  (min_score={self.min_score} gates on raw confidence only)\n'
            f'  panel_group_radius_m={self.panel_group_radius_m}\n'
            f'  team source: {self.ref_sys_topic}'
        )

    def on_ref_sys(self, msg):
        new_val = bool(msg.is_on_blue_team)
        if self.is_blue_team is None or self.is_blue_team != new_val:
            self.get_logger().info(
                f"Team colour set to {'BLUE' if new_val else 'RED'} "
                f"(excluding class IDs {'0-3' if new_val else '4-7'})")
        self.is_blue_team = new_val

    def on_array(self, msg):
        candidates = []
        n_filtered = 0
        for det in msg.detections:
            if not eligible(det.confidence, det.class_id, self.is_blue_team, self.min_score):
                n_filtered += 1
                continue
            centrality = centrality_3d(
                det.center.x, det.center.y, det.center.z, self.centrality_max_angle_rad)
            score = compute_score(
                det.confidence, centrality, det.class_id, self.center_weight,
                self.priority_class_bonus, self.priority_class_ids)
            candidates.append({
                'x': det.center.x, 'y': det.center.y, 'z': det.center.z,
                'score': score, 'det': det,
            })

        if not candidates:
            if self.is_blue_team is None and msg.detections:
                self.get_logger().warn(
                    f"No RefSysStatus on '{self.ref_sys_topic}' yet -- "
                    'team colour unknown, passing all detections through',
                    throttle_duration_sec=5.0)
            return

        clusters = group_panels(candidates, self.panel_group_radius_m)
        cluster_infos = []
        for indices in clusters:
            centroid = cluster_centroid(candidates, indices)
            # Best panel within this cluster, per-frame, no stickiness.
            best_idx = max(indices, key=lambda i: candidates[i]['score'])
            cluster_infos.append({
                'key': best_idx,
                'centroid': centroid,
                'score': candidates[best_idx]['score'],
            })

        winner_key = self.hysteresis.update(cluster_infos)
        if winner_key is None:
            return

        winner = candidates[winner_key]['det']
        winner.robot_track_id = self.hysteresis.track_id
        self.pub.publish(winner)


def main(args=None):
    rclpy.init(args=args)
    node = TargetSelector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
