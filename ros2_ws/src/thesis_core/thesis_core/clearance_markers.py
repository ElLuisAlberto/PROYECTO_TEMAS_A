"""RViz dimensions for instantaneous and nominal JOG proximity."""
import math

from builtin_interfaces.msg import Duration as DurationMessage
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def clearance_markers(header, name, hit, color, state, *, nominal=False):
    """Draw one measured gap line and one distance-only label."""
    axis = hit.closest_robot_point
    target = hit.box_point if hit.box_point is not None else hit.obstacle_center
    delta = tuple(b - a for a, b in zip(axis, target))
    length = math.hypot(*delta)
    direction = tuple(v / length for v in delta) if length else (0., 0., 1.)
    start = tuple(v + hit.capsule_radius * d for v, d in zip(axis, direction))
    end = target if hit.box_point is not None else tuple(
        v - hit.obstacle_radius * d for v, d in zip(target, direction))

    line = Marker(header=header, ns=name, id=0, type=Marker.LINE_LIST)
    line.pose.orientation.w = 1.
    line.scale.x = .008
    line.color.r, line.color.g, line.color.b, line.color.a = (*color, 1.)
    line.points = [Point(x=x, y=y, z=z) for x, y, z in (start, end)]
    line.lifetime = DurationMessage(nanosec=300000000)
    label_position = [
        (start[0] + end[0]) / 2,
        (start[1] + end[1]) / 2,
        (start[2] + end[2]) / 2 + .04,
    ]

    label = Marker(header=header, ns=name, id=1, type=Marker.TEXT_VIEW_FACING)
    label.pose.orientation.w = 1.
    label.pose.position = Point(
        x=label_position[0], y=label_position[1], z=label_position[2],
    )
    label.scale.z = .034
    label.color = line.color
    label.lifetime = line.lifetime
    label.text = f'{hit.clearance:.3f} m'
    return [line, label]
