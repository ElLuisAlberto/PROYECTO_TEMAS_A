"""CAD-style VTK camera controls; these never alter the robot or ROS state."""
from __future__ import annotations

import math
from typing import Callable

import numpy as np
import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor


def _rotation(axis, angle):
    """Right-handed Rodrigues rotation about a unit world-space axis."""
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z = axis / norm
    skew = np.array(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)))
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


class CadCameraStyle(vtk.vtkInteractorStyleUser):
    """Left drag orbits clicked geometry; middle drag pans; Shift+middle orbits.

    A rigid rotation of both camera position and focal point keeps the clicked
    pixel fixed while orbiting. Empty space uses the current focal point.
    Coordinates are VTK framebuffer pixels; sensitivity is normalized by DPR.
    """

    # Keep one full-width drag comfortably below a half turn. The previous
    # TrackballCamera base style also processed mouse events, effectively
    # adding its own much faster rotation to ours on some VTK builds.
    DEGREES_PER_LOGICAL_PIXEL = 0.10
    ZOOM_STEP = 1.12
    MIN_PARALLEL_SCALE = 0.12
    MAX_PARALLEL_SCALE = 5.0
    MIN_DISTANCE = 0.2
    MAX_DISTANCE = 20.0

    def __init__(self, renderer, pixel_ratio: Callable[[], float] = lambda: 1.0):
        super().__init__()
        self.renderer = renderer
        self.pixel_ratio = pixel_ratio
        self.picker = vtk.vtkCellPicker()
        self.picker.SetTolerance(0.0005)
        self._mode = None
        self._last = None
        self._pivot = None
        self.AddObserver("LeftButtonPressEvent", self._left_down)
        self.AddObserver("LeftButtonReleaseEvent", self._left_up)
        self.AddObserver("MiddleButtonPressEvent", self._middle_down)
        self.AddObserver("MiddleButtonReleaseEvent", self._middle_up)
        self.AddObserver("MouseMoveEvent", self._move)
        self.AddObserver("MouseWheelForwardEvent", self._wheel_forward)
        self.AddObserver("MouseWheelBackwardEvent", self._wheel_backward)

    def cancel_drag(self):
        self._mode = self._last = self._pivot = None

    def pick_pivot(self, x, y):
        """Pick visible scene geometry, otherwise retain the existing focus."""
        if self.picker.Pick(float(x), float(y), 0.0, self.renderer):
            point = tuple(float(v) for v in self.picker.GetPickPosition())
            if all(math.isfinite(v) for v in point):
                return point
        return tuple(self.renderer.GetActiveCamera().GetFocalPoint())

    def _begin(self, mode):
        interactor = self.GetInteractor()
        x, y = interactor.GetEventPosition()
        self._mode, self._last = mode, (x, y)
        self._pivot = self.pick_pivot(x, y) if mode == "orbit" else None

    def _left_down(self, _obj, _event):
        self._begin("orbit")

    def _left_up(self, _obj, _event):
        if self._mode == "orbit":
            self.cancel_drag()

    def _middle_down(self, _obj, _event):
        self._begin("orbit" if self.GetInteractor().GetShiftKey() else "pan")

    def _middle_up(self, _obj, _event):
        self.cancel_drag()

    def _move(self, _obj, _event):
        if self._mode is None or self._last is None:
            return
        current = self.GetInteractor().GetEventPosition()
        if current == self._last:
            return
        if self._mode == "orbit":
            try:
                ratio = float(self.pixel_ratio())
            except (TypeError, ValueError):
                ratio = 1.0
            if not math.isfinite(ratio) or ratio <= 0:
                ratio = 1.0
            self.orbit((current[0] - self._last[0]) / ratio,
                       (current[1] - self._last[1]) / ratio, self._pivot)
        else:
            self.pan(self._last, current)
        self._last = current
        self._render()

    def orbit(self, dx, dy, pivot):
        camera = self.renderer.GetActiveCamera()
        position = np.array(camera.GetPosition(), dtype=float)
        focal = np.array(camera.GetFocalPoint(), dtype=float)
        up = np.array(camera.GetViewUp(), dtype=float)
        pivot = np.array(pivot, dtype=float)
        if not all(np.all(np.isfinite(value)) for value in (position, focal, up, pivot)):
            return
        scale = math.radians(self.DEGREES_PER_LOGICAL_PIXEL)
        yaw = _rotation((0, 0, 1), -float(dx) * scale)
        direction = yaw @ (focal - position)
        rotated_up = yaw @ up
        right = np.cross(direction, rotated_up)
        pitch = _rotation(right, -float(dy) * scale)
        candidate = pitch @ direction
        length = np.linalg.norm(candidate)
        # Prevent upside-down camera flips near top and bottom views.
        if length < 1e-12 or abs(candidate[2] / length) > 0.995:
            pitch = np.eye(3)
        rotation = pitch @ yaw
        camera.SetPosition(*(pivot + rotation @ (position - pivot)))
        camera.SetFocalPoint(*(pivot + rotation @ (focal - pivot)))
        camera.SetViewUp(*(rotation @ up))
        camera.OrthogonalizeViewUp()

    def _world_at(self, x, y, depth):
        self.renderer.SetDisplayPoint(float(x), float(y), float(depth))
        self.renderer.DisplayToWorld()
        point = self.renderer.GetWorldPoint()
        if abs(point[3]) < 1e-12:
            return None
        return np.array(point[:3], dtype=float) / point[3]

    def _focal_depth(self):
        camera = self.renderer.GetActiveCamera()
        self.renderer.SetWorldPoint(*camera.GetFocalPoint(), 1.0)
        self.renderer.WorldToDisplay()
        return self.renderer.GetDisplayPoint()[2]

    def pan(self, previous, current):
        depth = self._focal_depth()
        before = self._world_at(*previous, depth)
        after = self._world_at(*current, depth)
        if before is None or after is None:
            return
        self._translate_camera(before - after)

    def _translate_camera(self, delta):
        camera = self.renderer.GetActiveCamera()
        camera.SetPosition(*(np.array(camera.GetPosition()) + delta))
        camera.SetFocalPoint(*(np.array(camera.GetFocalPoint()) + delta))

    def zoom_at(self, x, y, forward):
        camera = self.renderer.GetActiveCamera()
        depth = self._focal_depth()
        before = self._world_at(x, y, depth)
        if before is None:
            return
        factor = self.ZOOM_STEP if forward else 1.0 / self.ZOOM_STEP
        if camera.GetParallelProjection():
            old = camera.GetParallelScale()
            camera.SetParallelScale(max(self.MIN_PARALLEL_SCALE,
                                        min(self.MAX_PARALLEL_SCALE, old / factor)))
        else:
            old = camera.GetDistance()
            target = max(self.MIN_DISTANCE, min(self.MAX_DISTANCE, old / factor))
            camera.Dolly(old / target)
        after = self._world_at(x, y, self._focal_depth())
        if after is not None:
            self._translate_camera(before - after)

    def _wheel_forward(self, _obj, _event):
        self.zoom_at(*self.GetInteractor().GetEventPosition(), True)
        self._render()

    def _wheel_backward(self, _obj, _event):
        self.zoom_at(*self.GetInteractor().GetEventPosition(), False)
        self._render()

    def _render(self):
        self.renderer.ResetCameraClippingRange()
        self.GetInteractor().GetRenderWindow().Render()


class CadVTKWidget(QVTKRenderWindowInteractor):
    """VTK's Qt adapter does not update event coordinates on wheel events."""

    def wheelEvent(self, event):
        ctrl, shift = self._GetCtrlShift(event)
        self._setEventInformation(event.x(), event.y(), ctrl, shift, chr(0), 0, None)
        super().wheelEvent(event)

    def focusOutEvent(self, event):
        style = self._Iren.GetInteractorStyle()
        if hasattr(style, "cancel_drag"):
            style.cancel_drag()
        super().focusOutEvent(event)
