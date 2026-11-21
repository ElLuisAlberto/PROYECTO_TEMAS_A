"""Horizontal work surface shared by the scene and both clearance paths."""
from dataclasses import dataclass, replace
import math

from thesis_core.obstacle_contract import rotate_vector
from thesis_core.jaco_kinematics import CLEARANCE_EXCLUDED_SEGMENT_NAMES

from thesis_core.clearance_geometry import finite_point, segment_box_clearance


TABLE_SIZE = (1.20, 0.80, 0.04)
JACO2_SIM_TABLE_SIZE = (1.60, 1.00, 0.04)
MOUNT_HEIGHT = 0.75
TABLE_CENTER = (0.0, 0.0, MOUNT_HEIGHT - TABLE_SIZE[2] / 2)
SPHERE_HEIGHT = MOUNT_HEIGHT + 0.40 + 0.12


def fixed_mount_links(root):
    """Links rigidly attached to world may touch their own mounting surface."""
    fixed = [(j.find('parent').get('link'), j.find('child').get('link'))
             for j in root.findall('joint') if j.get('type') == 'fixed']
    links = {'world'}
    while True:
        children = {child for parent, child in fixed if parent in links}
        if children <= links:
            return links
        links.update(children)


@dataclass(frozen=True)
class WorkTable:
    center: tuple
    size: tuple
    excluded: frozenset
    orientation: tuple = (0., 0., 0., 1.)

    with_legs: bool = False

    def leg_boxes(self, center):
        height = MOUNT_HEIGHT - self.size[2]
        for x in (-1, 1):
            for y in (-1, 1):
                yield ((center[0] + x * (self.size[0] / 2 - .08),
                        center[1] + y * (self.size[1] / 2 - .08),
                        center[2] - (self.size[2] + height) / 2),
                       (.06, .06, height))

    def __post_init__(self):
        finite_point(self.center, 'table center')
        if len(self.orientation) != 4 or not math.isclose(
                math.hypot(*self.orientation), 1.0, abs_tol=1e-8):
            raise ValueError('table orientation must be a unit quaternion')
        if min(finite_point(self.size, 'table size')) <= 0:
            raise ValueError('table size must be positive')

    def closest(self, segments, names, radii, sample_index=0, sample_time=0.0):
        result = None
        rotated = self.orientation != (0., 0., 0., 1.)
        center = (0., 0., 0.) if rotated else self.center
        top = center[2] + self.size[2] / 2
        if rotated:
            x, y, z, w = self.orientation

            def local(point):
                offset = tuple(v - c for v, c in zip(point, self.center))
                return rotate_vector(offset, (-x, -y, -z, w))
        for i, ((a, b), name, radius) in enumerate(zip(segments, names, radii)):
            if name in self.excluded:
                continue
            if rotated:
                a, b = local(a), local(b)
            # Vertical clearance is a cheap lower bound above the board.
            if result is not None and min(a[2], b[2]) - top - radius >= result.clearance:
                continue
            box = segment_box_clearance(a, b, radius, center, self.size,
                                        name, i, sample_index=sample_index,
                                        sample_time=sample_time)
            # Above the board its surface is always at least as close as a leg.
            if self.with_legs and min(a[2], b[2]) < top - self.size[2]:
                for leg_center, leg_size in self.leg_boxes(center):
                    leg = segment_box_clearance(a, b, radius, leg_center, leg_size,
                                                name, i, sample_index=sample_index,
                                                sample_time=sample_time)
                    if leg.clearance < box.clearance:
                        box = leg
            if result is None or box.clearance < result.clearance:
                result = box
        if result is None:
            raise ValueError("Table checks require at least one moving capsule")
        if rotated:
            contact = rotate_vector(result.box_point, self.orientation)
            offset = rotate_vector(result.closest_robot_point, self.orientation)
            a, b = segments[result.segment_index]
            result = replace(result, capsule_start=a, capsule_end=b,
                             box_point=tuple(v + c for v, c in zip(contact, self.center)),
                             obstacle_center=self.center,
                             closest_robot_point=tuple(v + c for v, c in zip(offset, self.center)))
        return result

    def minimum(self, sphere, segments, names, radii):
        box = self.closest(segments, names, radii, sphere.sample_index, sphere.sample_time)
        return min((sphere, box), key=lambda hit: hit.clearance)


def configured_table(node, profile=None):
    enabled = node.declare_parameter('table_enabled', False).value
    center = node.declare_parameter('table_center', list(TABLE_CENTER)).value
    size = node.declare_parameter('table_size', list(TABLE_SIZE)).value
    excluded = node.declare_parameter(
        'table_excluded_segments', list(CLEARANCE_EXCLUDED_SEGMENT_NAMES)
    ).value
    orientation = node.declare_parameter('table_orientation', [0., 0., 0., 1.]).value
    with_legs = node.declare_parameter('table_with_legs', False).value
    if not enabled:
        return None
    if profile is None and set(excluded) != set(CLEARANCE_EXCLUDED_SEGMENT_NAMES):
        raise ValueError(
            'Historical JACO2 table checks must exclude the two configured '
            'base capsules'
        )
    if profile is not None:
        mount = fixed_mount_links(profile.urdf_root)
        excluded = [c.name for c in profile.collision_capsules if c.link in mount]
    return WorkTable(tuple(center), tuple(size), frozenset(excluded), tuple(orientation), with_legs)


def table_marker(table, header):
    from visualization_msgs.msg import Marker
    marker = Marker()
    marker.header = header
    marker.ns = 'work_table'
    marker.type = Marker.CUBE
    q = marker.pose.orientation
    q.x, q.y, q.z, q.w = table.orientation
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = table.center
    marker.scale.x, marker.scale.y, marker.scale.z = table.size
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.52, 0.31, 0.15, 0.8)
    return marker
