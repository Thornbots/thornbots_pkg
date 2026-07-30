"""
target_selector_core.py — pure scoring/centrality/grouping/hysteresis logic
for target_selector.py, with no rclpy or ROS message imports so it's
unit-testable standalone (test/test_target_selector.py) without a built
workspace. See sentry_pkg/README.md's ### target_selector.py Notes for the
clustering-rule tradeoff and scoring history.
"""
import math


def is_excluded_by_team(class_id, is_blue_team):
    """True if class_id belongs to our own team (mirrors the old picker's
    isExcludedByTeam). is_blue_team=None (no RefSysStatus yet) never
    excludes, so the pipeline runs before the first status arrives."""
    if is_blue_team is None:
        return False
    if is_blue_team:
        return 0 <= class_id <= 3
    return 4 <= class_id <= 7


def centrality_3d(x, y, z, max_angle_rad):
    """Bearing off the camera's forward (+x) axis, 1.0 at boresight, 0.0 at
    max_angle_rad or beyond. Replaces the old picker's image-plane-distance
    centrality now that panels are deprojected to 3D: this is the post-depth
    equivalent, not a port of the pixel formula (deliberate, see the plan's
    Phase 1 section). x<=0 (behind the camera) scores 0 rather than a
    divide-by-zero/undefined atan2 case."""
    if x <= 0.0 or max_angle_rad <= 0.0:
        return 0.0
    angle = math.atan2(math.hypot(y, z), x)
    c = 1.0 - angle / max_angle_rad
    return max(0.0, min(1.0, c))


def compute_score(confidence, centrality, class_id, center_weight,
                   priority_class_bonus, priority_class_ids):
    """score = confidence + center_weight*centrality + priority_class_bonus
    (added conditionally, NOT confidence*priority_class_bonus or any other
    multiplicative form -- see detection_picker_node.cpp:281-284, which this
    ports exactly)."""
    score = confidence + center_weight * centrality
    if class_id in priority_class_ids:
        score += priority_class_bonus
    return score


def eligible(confidence, class_id, is_blue_team, min_score):
    """min_score gates on raw confidence only (detection_picker_node.cpp:277
    and its comment at :38-39) -- centrality/priority bonus never resurrect
    a low-confidence detection, they only re-rank ones already eligible."""
    if is_excluded_by_team(class_id, is_blue_team):
        return False
    return confidence >= min_score


def group_panels(panels, radius_m):
    """Single-linkage clustering of panels (each a dict with 'x','y','z')
    into robots, by 3D distance in the shared camera frame. All panels in
    one PanelDetectionArray share one camera pose, so camera-frame Euclidean
    distance equals true metric distance -- no need to transform to odom.

    Single-linkage (not centroid) chosen deliberately: transitivity reaches
    all 4 panels of one spinning robot via its adjacent-pair chain (~0.384m
    apart) even though only 1-2 panels are usually visible at once; the
    tradeoff is it also merges two robots whose nearest panels fall within
    radius_m. See README.md's ### target_selector.py Notes.

    Returns a list of clusters, each a list of indices into `panels`.
    """
    n = len(panels)
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(i, j):
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[ri] = rj

    for i in range(n):
        for j in range(i + 1, n):
            dx = panels[i]['x'] - panels[j]['x']
            dy = panels[i]['y'] - panels[j]['y']
            dz = panels[i]['z'] - panels[j]['z']
            if math.sqrt(dx * dx + dy * dy + dz * dz) <= radius_m:
                union(i, j)

    clusters = {}
    for i in range(n):
        clusters.setdefault(find(i), []).append(i)
    return list(clusters.values())


def cluster_centroid(panels, indices):
    n = len(indices)
    cx = sum(panels[i]['x'] for i in indices) / n
    cy = sum(panels[i]['y'] for i in indices) / n
    cz = sum(panels[i]['z'] for i in indices) / n
    return (cx, cy, cz)


class RobotHysteresis:
    """Robot-level hysteresis (never panel-level -- panels of a spinning
    robot legitimately vanish every 0.5-1s, so panel stickiness would delay
    every correct handoff). A challenger cluster must beat the incumbent's
    score by switch_margin for switch_hold_frames consecutive frames before
    the incumbent switches. Track continuity across frames is by nearest
    centroid to the current incumbent (clusters have no persistent identity
    otherwise) -- acquiring from no-incumbent is immediate, only switching
    between two simultaneously-visible candidates is delayed.
    """

    def __init__(self, switch_margin, switch_hold_frames, gate_radius_m=1.0):
        self.switch_margin = switch_margin
        self.switch_hold_frames = switch_hold_frames
        self.gate_radius_m = gate_radius_m
        self.track_id = 0  # 0 = no incumbent yet
        self.incumbent_centroid = None
        self.incumbent_score = None
        self._challenge_streak = 0
        self._next_id = 1

    def update(self, clusters):
        """clusters: list of dict(centroid=(x,y,z), score=float, key=any).
        Returns the winning cluster's `key`, or None if clusters is empty."""
        if not clusters:
            return None

        best = max(clusters, key=lambda c: c['score'])

        if self.incumbent_centroid is None:
            self._acquire(best)
            return best['key']

        # Nearest cluster to the incumbent's last known position -- may or
        # may not be `best`.
        def dist(c):
            cx, cy, cz = c['centroid']
            ix, iy, iz = self.incumbent_centroid
            return math.sqrt((cx - ix) ** 2 + (cy - iy) ** 2 + (cz - iz) ** 2)

        incumbent_match = min(clusters, key=dist)
        if dist(incumbent_match) > self.gate_radius_m:
            # Incumbent lost track entirely -- re-acquire immediately.
            self._acquire(best)
            return best['key']

        if best is incumbent_match or best['score'] <= incumbent_match['score']:
            self._challenge_streak = 0
            self._track(incumbent_match)
            return incumbent_match['key']

        if best['score'] >= incumbent_match['score'] + self.switch_margin:
            self._challenge_streak += 1
            if self._challenge_streak >= self.switch_hold_frames:
                self._acquire(best)
                return best['key']
        else:
            self._challenge_streak = 0

        self._track(incumbent_match)
        return incumbent_match['key']

    def _acquire(self, cluster):
        self.track_id = self._next_id
        self._next_id += 1
        self._challenge_streak = 0
        self._track(cluster)

    def _track(self, cluster):
        self.incumbent_centroid = cluster['centroid']
        self.incumbent_score = cluster['score']
