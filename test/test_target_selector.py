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
Unit tests for target_selector_core.py's scoring/centrality/grouping/hysteresis.

Synthetic inputs only -- not a live side-by-side against the old
detection_picker_node (it consumed 2D pre-depth detections and the new
selector consumes 3D post-depth ones, so "identical picks" isn't well
defined between them). Imports only target_selector_core (no rclpy, no
ROS message packages), so this runs on a bare Python 3 + pytest install
with no workspace build. Run with
`python3 -m pytest test/test_target_selector.py`.
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


def test_score_center_weight_scales_only_the_centrality_term():
    # Both existing cases used center_weight=1.0, where confidence and
    # centrality are interchangeable and the weight is invisible. At 0.3
    # applying it to confidence (or to the sum, or to the bonus) all give
    # different numbers than applying it to centrality alone.
    s = compute_score(0.8, 0.5, class_id=0, center_weight=0.3,
                      priority_class_bonus=0.5, priority_class_ids={2, 6})
    assert math.isclose(s, 0.8 + 0.3 * 0.5)
    # A weight of 0 must drop centrality entirely, not the confidence.
    s_zero = compute_score(0.8, 0.5, class_id=0, center_weight=0.0,
                           priority_class_bonus=0.5, priority_class_ids={2, 6})
    assert math.isclose(s_zero, 0.8)
    # ...and the bonus is unweighted: it adds in full at any center_weight.
    s_bonus = compute_score(0.8, 0.5, class_id=2, center_weight=0.3,
                            priority_class_bonus=0.5, priority_class_ids={2, 6})
    assert math.isclose(s_bonus, s + 0.5)


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


def test_group_panels_links_at_exactly_the_radius_and_not_beyond():
    # The old cases sat at 0.384m and 3.0m against a 0.4m radius -- nothing
    # near the boundary, so a radius/diameter mixup was invisible. These
    # straddle it.
    #
    # The separations are a 3-4-5 triple scaled by 0.08, so the distance is
    # *exactly* 0.4 in binary floating point (0.24^2 + 0.32^2 == 0.16, and
    # sqrt of that is 0.4 with no residual). A naive 2.0-to-2.4 pair does
    # not work here: it lands a half-ulp short of 0.4, so it links under
    # both `<` and `<=` and pins nothing about the comparison itself.
    at_radius = [{'x': 0.0, 'y': 0.0, 'z': 0.0}, {'x': 0.24, 'y': 0.32, 'z': 0.0}]
    assert len(group_panels(at_radius, radius_m=0.4)) == 1

    beyond = [{'x': 0.0, 'y': 0.0, 'z': 0.0}, {'x': 0.25, 'y': 0.33, 'z': 0.0}]
    assert len(group_panels(beyond, radius_m=0.4)) == 2


def test_group_panels_merges_two_robots_with_close_nearest_panels():
    # Pins the tradeoff the docstring calls out, so it stays a known cost
    # of single-linkage rather than a surprise: two distinct robots 0.3m
    # apart at their nearest panels come back as ONE cluster. If this ever
    # needs to stop being true, single-linkage is what has to change.
    robot_a = [{'x': 2.0, 'y': 0.0, 'z': 0.0}, {'x': 1.7, 'y': 0.24, 'z': 0.0}]
    robot_b = [{'x': 2.3, 'y': 0.0, 'z': 0.0}, {'x': 2.6, 'y': 0.24, 'z': 0.0}]
    clusters = group_panels(robot_a + robot_b, radius_m=0.4)
    assert len(clusters) == 1
    assert sorted(clusters[0]) == [0, 1, 2, 3]

    # Transitivity is the mechanism: the chain only spans the gap because
    # of the 0.3m bridge. Widen it past the radius and they separate.
    robot_b_far = [{'x': 2.5, 'y': 0.0, 'z': 0.0}, {'x': 2.8, 'y': 0.24, 'z': 0.0}]
    assert len(group_panels(robot_a + robot_b_far, radius_m=0.4)) == 2


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


def test_hysteresis_streak_resets_when_challenger_margin_lapses():
    # The flicker-resistance path: a challenger must clear switch_margin on
    # CONSECUTIVE frames. One sub-margin frame in the middle sends the
    # streak back to 0, so the earlier frames don't count toward the next
    # switch. Without the reset the switch would land on frame 4 below.
    h = RobotHysteresis(switch_margin=0.3, switch_hold_frames=3)
    h.update([{'key': 'a', 'centroid': (2.0, 0.0, 0.0), 'score': 1.0}])
    first_id = h.track_id

    def frame(challenger_score):
        return h.update([
            {'key': 'a', 'centroid': (2.0, 0.0, 0.0), 'score': 1.0},
            {'key': 'b', 'centroid': (2.0, 5.0, 0.0), 'score': challenger_score},
        ])

    assert frame(2.0) == 'a'    # streak 1
    assert frame(1.1) == 'a'    # ahead but under margin -> streak back to 0
    assert frame(2.0) == 'a'    # streak 1 again
    assert frame(2.0) == 'a'    # streak 2 -- would have been the switch
    assert h.track_id == first_id
    assert frame(2.0) == 'b'    # streak 3 -> switch
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
