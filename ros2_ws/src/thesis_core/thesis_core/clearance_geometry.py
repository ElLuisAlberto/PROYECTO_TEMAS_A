"""Pure capsule-sphere clearance primitives for the thesis geometry."""

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence, Tuple


Point3 = Tuple[float, float, float]
Segment3 = Tuple[Point3, Point3]
DEGENERATE_SEGMENT_EPSILON_SQ = 1.0e-12


@dataclass(frozen=True)
class SegmentClearance:
    """Minimum surface clearance and its complete geometric witnesses."""

    clearance: float
    segment_name: str
    segment_index: int
    sample_index: int
    sample_time: float
    capsule_start: Point3
    capsule_end: Point3
    closest_robot_point: Point3
    obstacle_center: Point3
    capsule_radius: float
    obstacle_radius: float
    box_point: Point3 | None = None

    def legacy_tuple(
        self,
    ) -> Tuple[float, str, int, Point3, Point3]:
        """Return the tuple historically consumed by the supervisor."""
        return (
            self.clearance,
            self.segment_name,
            self.segment_index,
            self.capsule_start,
            self.capsule_end,
        )


def finite_point(values: Sequence[float], name: str) -> Point3:
    """Return one finite three-dimensional point."""
    result = tuple(float(value) for value in values)
    if len(result) != 3 or not all(math.isfinite(value) for value in result):
        raise ValueError(f'{name} must contain three finite values')
    return result


def positive_radius(value: float, name: str) -> float:
    """Return one finite strictly positive radius."""
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f'{name} must be finite and greater than zero')
    return result


def closest_point_on_segment(
    point: Sequence[float],
    start: Sequence[float],
    end: Sequence[float],
) -> Point3:
    """Project a point onto a finite segment, including degenerate axes."""
    query = finite_point(point, 'point')
    first = finite_point(start, 'segment start')
    second = finite_point(end, 'segment end')
    axis = tuple(second[i] - first[i] for i in range(3))
    relative = tuple(query[i] - first[i] for i in range(3))
    denominator = sum(value * value for value in axis)
    if denominator <= DEGENERATE_SEGMENT_EPSILON_SQ:
        return first
    factor = sum(relative[i] * axis[i] for i in range(3)) / denominator
    factor = max(0.0, min(1.0, factor))
    return tuple(first[i] + factor * axis[i] for i in range(3))


def point_distance(first: Sequence[float], second: Sequence[float]) -> float:
    """Return finite Euclidean point distance."""
    point_a = finite_point(first, 'first point')
    point_b = finite_point(second, 'second point')
    return math.dist(point_a, point_b)


def segment_sphere_clearance(
    start: Sequence[float],
    end: Sequence[float],
    capsule_radius: float,
    obstacle_center: Sequence[float],
    obstacle_radius: float,
    segment_name: str = 'segment',
    segment_index: int = 0,
    sample_index: int = 0,
    sample_time: float = 0.0,
) -> SegmentClearance:
    """Calculate surface clearance between one capsule and one sphere."""
    first = finite_point(start, 'capsule start')
    second = finite_point(end, 'capsule end')
    center = finite_point(obstacle_center, 'obstacle center')
    capsule = positive_radius(capsule_radius, 'capsule radius')
    obstacle = positive_radius(obstacle_radius, 'obstacle radius')
    time_value = float(sample_time)
    if not math.isfinite(time_value) or time_value < 0.0:
        raise ValueError('sample_time must be finite and non-negative')
    if int(sample_index) < 0 or int(segment_index) < 0:
        raise ValueError('indices must be non-negative')
    if not str(segment_name):
        raise ValueError('segment_name must not be empty')
    closest = closest_point_on_segment(center, first, second)
    clearance = point_distance(center, closest) - capsule - obstacle
    return SegmentClearance(
        clearance=clearance,
        segment_name=str(segment_name),
        segment_index=int(segment_index),
        sample_index=int(sample_index),
        sample_time=time_value,
        capsule_start=first,
        capsule_end=second,
        closest_robot_point=closest,
        obstacle_center=center,
        capsule_radius=capsule,
        obstacle_radius=obstacle,
    )


def segment_box_clearance(
    start: Sequence[float],
    end: Sequence[float],
    capsule_radius: float,
    box_center: Sequence[float],
    box_size: Sequence[float],
    segment_name: str = 'segment',
    segment_index: int = 0,
    samples: int = 128,
    sample_index: int = 0,
    sample_time: float = 0.0,
) -> SegmentClearance:
    """Exact capsule-axis distance to a solid box, less capsule radius."""
    first = finite_point(start, 'segment start')
    second = finite_point(end, 'segment end')
    center = finite_point(box_center, 'box center')
    size = finite_point(box_size, 'box size')
    capsule = positive_radius(capsule_radius, 'capsule radius')
    if any(value <= 0.0 for value in size):
        raise ValueError('box size must be positive')
    if int(samples) < 2:
        raise ValueError('samples must be at least two')
    half = tuple(value / 2.0 for value in size)
    lower = tuple(center[index] - half[index] for index in range(3))
    upper = tuple(center[index] + half[index] for index in range(3))
    axis = tuple(second[index] - first[index] for index in range(3))
    # Between face crossings the squared distance is a convex quadratic.
    # Its minimum is either at the stationary point or an interval endpoint.
    cuts = {0.0, 1.0}
    for i, delta in enumerate(axis):
        if delta:
            for face in (lower[i], upper[i]):
                t = (face - first[i]) / delta
                if 0 < t < 1:
                    cuts.add(t)
    cuts = sorted(cuts)
    distance_sq = math.inf
    for left, right in zip(cuts, cuts[1:]):
        mid = (left + right) * 0.5
        slope = curvature = 0.0
        for i, delta in enumerate(axis):
            value = first[i] + delta * mid
            if lower[i] <= value <= upper[i]:
                continue
            face = lower[i] if value < lower[i] else upper[i]
            slope += (first[i] - face) * delta
            curvature += delta * delta
        t = max(left, min(right, -slope / curvature)) if curvature else left
        point = tuple(first[i] + t * axis[i] for i in range(3))
        squared = sum(max(lower[i]-v, v-upper[i], 0.0)**2
                      for i, v in enumerate(point))
        if squared < distance_sq:
            closest, distance_sq = point, squared
    clearance = math.sqrt(distance_sq) - capsule
    surface = [max(lo, min(v, hi)) for v, lo, hi in zip(closest, lower, upper)]
    # A capsule axis inside the solid has zero distance to the box.
    # Keep that signed clearance and place its visual witness on a face.
    if all(lo < v < hi for v, lo, hi in zip(closest, lower, upper)):
        _, i, face = min((abs(closest[i] - face), i, face)
                         for i in range(3) for face in (lower[i], upper[i]))
        surface[i] = face
    return SegmentClearance(
        clearance=clearance,
        segment_name=str(segment_name),
        segment_index=int(segment_index),
        sample_index=sample_index,
        sample_time=sample_time,
        capsule_start=first,
        capsule_end=second,
        closest_robot_point=closest,
        obstacle_center=center,
        box_point=tuple(surface),
        capsule_radius=capsule,
        # The message schema has a scalar obstacle radius.  Preserve a
        # conservative bounding radius for consumers that expose it.
        obstacle_radius=math.sqrt(sum(value * value for value in half)),
    )


def minimum_configuration_clearance(
    segments: Iterable[Segment3],
    segment_names: Sequence[str],
    capsule_radii: Sequence[float],
    obstacle_center: Sequence[float],
    obstacle_radius: float,
    sample_index: int = 0,
    sample_time: float = 0.0,
    *,
    included_segment_names: Optional[Sequence[str]] = None,
) -> SegmentClearance:
    """Evaluate selected capsules and preserve each winner's original index."""
    axes = tuple(segments)
    names = tuple(str(value) for value in segment_names)
    radii = tuple(float(value) for value in capsule_radii)
    if not axes or len(axes) != len(names) or len(axes) != len(radii):
        raise ValueError('segments, names and radii must have equal size')
    if included_segment_names is None:
        included = None
    else:
        requested = frozenset(str(value) for value in included_segment_names)
        if not requested or requested.difference(names):
            raise ValueError('included segment names must select known capsules')
        included = requested
    results = tuple(
        segment_sphere_clearance(
            start,
            end,
            radius,
            obstacle_center,
            obstacle_radius,
            name,
            index,
            sample_index,
            sample_time,
        )
        for index, ((start, end), name, radius) in enumerate(
            zip(axes, names, radii)
        )
        if included is None or name in included
    )
    return min(results, key=lambda result: result.clearance)
