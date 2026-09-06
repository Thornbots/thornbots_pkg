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
Unit tests for target_selector_core.py's pure scoring/centrality/grouping.

Unit tests for target_selector_core.py's pure scoring/centrality/grouping/
hysteresis functions, against synthetic inputs -- not a live side-by-side
against the old detection_picker_node (it consumed 2D pre-depth
detections and the new selector consumes 3D post-depth ones, so "identical
picks" isn't well defined between them). Imports only
target_selector_core (no rclpy, no ROS message packages), so this runs on
a bare Python 3 + pytest install with no
workspace build. Run with `python3 -m pytest test/test_target_selector.py`.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from thornbots_pkg.target_selector_core import (  # noqa: E402
    centrality_3d, cluster_centroid, compute_score, eligible,
    group_panels, is_excluded_by_team, RobotHysteresis,
)


# ── is_excluded_by_team ──────────────────────────────────────────────────

def test_team_filter_unknown_passes_everything():
    assert not is_excluded_by_team(0, None)
    assert not is_excluded_by_team(7, None)


def test_team_filter_blue_excludes_0_to_3():
    for cid in range(0, 4):
        assert is_excluded_by_team(cid, True)
    for cid in range(4, 8):
        assert not is_excluded_by_team(cid, True)


def test_team_filter_red_excludes_4_to_7():
    for cid in range(0, 4):
        assert not is_excluded_by_team(cid, False)
    for cid in range(4, 8):
        assert is_excluded_by_team(cid, False)


# ── centrality_3d ─────────────────────────────────────────────────────────

def test_centrality_boresight_is_one():
    assert centrality_3d(2.0, 0.0, 0.0, math.radians(45.0)) == 1.0


def test_centrality_behind_camera_is_zero():
    assert centrality_3d(-1.0, 0.0, 0.0, math.radians(45.0)) == 0.0
    assert centrality_3d(0.0, 0.0, 0.0, math.radians(45.0)) == 0.0


def test_centrality_clamped_at_max_angle():
    max_angle = math.radians(45.0)
    # Exactly at the max angle -> 0
    x = 1.0
    y = x * math.tan(max_angle)
    assert abs(centrality_3d(x, y, 0.0, max_angle)) < 1e-9
    # Beyond it -> still clamped to 0, not negative
    y_beyond = x * math.tan(max_angle * 2.0)
    assert centrality_3d(x, y_beyond, 0.0, max_angle) == 0.0


def test_centrality_monotonic_in_angle():
    max_angle = math.radians(45.0)
    c_near = centrality_3d(2.0, 0.2, 0.0, max_angle)
    c_far = centrality_3d(2.0, 1.0, 0.0, max_angle)
    assert 0.0 < c_far < c_near < 1.0


# ── compute_score ─────────────────────────────────────────────────────────

def test_score_is_additive_not_multiplicative():
    # priority bonus must be a conditional ADD, not conf*bonus or
    # centrality*bonus -- see detection_picker_node.cpp:281-284.
    base = compute_score(0.8, 0.5, class_id=0, center_weight=1.0,
                         priority_class_bonus=0.5, priority_class_ids={2, 6})
    with_bonus = compute_score(0.8, 0.5, class_id=2, center_weight=1.0,
                               priority_class_bonus=0.5, priority_class_ids={2, 6})
    assert with_bonus == base + 0.5
    assert base == 0.8 + 1.0 * 0.5


def test_score_priority_bonus_only_for_listed_classes():
    s = compute_score(0.5, 0.5, class_id=3, center_weight=1.0,
                      priority_class_bonus=0.5, priority_class_ids={2, 6})
    assert s == 1.0  # no bonus applied


# ── eligible (min_score gates on raw confidence only) ────────────────────

def test_min_score_gates_on_raw_confidence_not_composite():
    # High centrality/priority class, but confidence below min_score: must
    # still be rejected -- centrality/bonus never resurrect low confidence.
    assert not eligible(confidence=0.1, class_id=2, is_blue_team=None, min_score=0.5)
    assert eligible(confidence=0.6, class_id=2, is_blue_team=None, min_score=0.5)


def test_eligible_respects_team_exclusion_even_above_min_score():
    assert not eligible(confidence=0.99, class_id=1, is_blue_team=True, min_score=0.0)
    assert eligible(confidence=0.99, class_id=5, is_blue_team=True, min_score=0.0)


# ── group_panels (single-linkage clustering) ──────────────────────────────

def test_group_panels_single_robot_four_panels_one_cluster():
    # Roughly a 0.30 x 0.24 chassis footprint, adjacent-pair spacing ~0.384m.
    panels = [
        {'x': 2.0, 'y': 0.0, 'z': 0.0},     # front
        {'x': 1.7, 'y': 0.24, 'z': 0.0},    # left
        {'x': 1.4, 'y': 0.0, 'z': 0.0},     # back
        {'x': 1.7, 'y': -0.24, 'z': 0.0},   # right
    ]
    clusters = group_panels(panels, radius_m=0.4)
    assert len(clusters) == 1
    assert sorted(clusters[0]) == [0, 1, 2, 3]


def test_group_panels_two_far_apart_robots_stay_separate():
    panels = [
        {'x': 2.0, 'y': 0.0, 'z': 0.0},
        {'x': 2.0, 'y': 3.0, 'z': 0.0},
    ]
    clusters = group_panels(panels, radius_m=0.4)
    assert len(clusters) == 2


def test_cluster_centroid_is_mean_position():
    panels = [{'x': 0.0, 'y': 0.0, 'z': 0.0}, {'x': 2.0, 'y': 0.0, 'z': 0.0}]
    cx, cy, cz = cluster_centroid(panels, [0, 1])
    assert (cx, cy, cz) == (1.0, 0.0, 0.0)


# ── RobotHysteresis ────────────────────────────────────────────────────────

def test_hysteresis_acquires_immediately_with_no_incumbent():
    h = RobotHysteresis(switch_margin=0.3, switch_hold_frames=3)
    clusters = [{'key': 'a', 'centroid': (2.0, 0.0, 0.0), 'score': 1.0}]
    winner = h.update(clusters)
    assert winner == 'a'
    assert h.track_id == 1


def test_hysteresis_holds_incumbent_below_switch_margin():
    h = RobotHysteresis(switch_margin=0.3, switch_hold_frames=2)
    h.update([{'key': 'a', 'centroid': (2.0, 0.0, 0.0), 'score': 1.0}])
    first_id = h.track_id
    # Challenger only slightly better -- below switch_margin, must not switch.
    for _ in range(5):
        winner = h.update([
            {'key': 'a', 'centroid': (2.0, 0.0, 0.0), 'score': 1.0},
            {'key': 'b', 'centroid': (2.0, 5.0, 0.0), 'score': 1.1},
        ])
    assert winner == 'a'
    assert h.track_id == first_id


def test_hysteresis_switches_after_sustained_stronger_challenger():
    h = RobotHysteresis(switch_margin=0.3, switch_hold_frames=3)
    h.update([{'key': 'a', 'centroid': (2.0, 0.0, 0.0), 'score': 1.0}])
    first_id = h.track_id

    winner = None
    for _ in range(3):
        winner = h.update([
            {'key': 'a', 'centroid': (2.0, 0.0, 0.0), 'score': 1.0},
            {'key': 'b', 'centroid': (2.0, 5.0, 0.0), 'score': 2.0},
        ])
    assert winner == 'b'
    assert h.track_id == first_id + 1


def test_hysteresis_reacquires_when_incumbent_lost():
    h = RobotHysteresis(switch_margin=0.3, switch_hold_frames=3, gate_radius_m=1.0)
    h.update([{'key': 'a', 'centroid': (2.0, 0.0, 0.0), 'score': 1.0}])
    first_id = h.track_id
    # Only candidate now is far outside gate_radius_m of the old incumbent.
    winner = h.update([{'key': 'c', 'centroid': (2.0, 10.0, 0.0), 'score': 0.5}])
    assert winner == 'c'
    assert h.track_id == first_id + 1


def test_hysteresis_empty_clusters_returns_none():
    h = RobotHysteresis(switch_margin=0.3, switch_hold_frames=3)
    assert h.update([]) is None


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
