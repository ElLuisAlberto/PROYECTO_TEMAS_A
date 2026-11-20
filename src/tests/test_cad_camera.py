"""Camera navigation must preserve the selected point without robot control."""
import unittest

try:
    import numpy as np
    import vtk
    from teleop_cooperativo.cad_camera import CadCameraStyle
except ImportError as error:
    raise unittest.SkipTest(f"VTK/Qt unavailable: {error}")


class CadCameraTest(unittest.TestCase):
    def setUp(self):
        self.renderer = vtk.vtkRenderer()
        self.window = vtk.vtkRenderWindow()
        self.window.SetSize(800, 600)
        self.window.AddRenderer(self.renderer)
        self.camera = self.renderer.GetActiveCamera()
        self.camera.SetPosition(1.8, -2.4, 1.65)
        self.camera.SetFocalPoint(0.22, 0.0, 0.25)
        self.camera.SetViewUp(0, 0, 1)
        self.camera.ParallelProjectionOn()
        self.camera.SetParallelScale(0.95)
        self.style = CadCameraStyle(self.renderer)

    def project(self, point):
        self.renderer.SetWorldPoint(*point, 1.0)
        self.renderer.WorldToDisplay()
        return np.array(self.renderer.GetDisplayPoint())

    def test_orbit_keeps_clicked_point_in_place_without_jump(self):
        pivot = (0.2, 0.0, 0.4)
        position = np.array(self.camera.GetPosition())
        focal = np.array(self.camera.GetFocalPoint())
        before = self.project(pivot)
        self.style.orbit(0, 0, pivot)
        np.testing.assert_allclose(self.camera.GetPosition(), position, atol=1e-12)
        np.testing.assert_allclose(self.camera.GetFocalPoint(), focal, atol=1e-12)
        self.style.orbit(40, 20, pivot)
        np.testing.assert_allclose(self.project(pivot)[:2], before[:2], atol=1e-7)
        self.assertAlmostEqual(np.linalg.norm(position - pivot),
                               np.linalg.norm(np.array(self.camera.GetPosition()) - pivot))

    def test_style_does_not_enable_vtks_default_trackball_camera(self):
        """Mouse motion is owned solely by the CAD handlers in this module."""
        self.assertNotIsInstance(self.style, vtk.vtkInteractorStyleTrackballCamera)

    def test_orbit_sensitivity_is_deliberately_slow(self):
        self.assertLessEqual(self.style.DEGREES_PER_LOGICAL_PIXEL, 0.10)

    def test_blank_falls_back_to_camera_focus(self):
        self.assertEqual(self.style.pick_pivot(799, 599), self.camera.GetFocalPoint())

    def test_zoom_preserves_cursor_world_point_and_respects_limits(self):
        before = self.style._world_at(617, 234, self.style._focal_depth())
        screen = self.project(before)
        self.style.zoom_at(617, 234, True)
        np.testing.assert_allclose(self.project(before)[:2], screen[:2], atol=1e-7)
        for _ in range(100):
            self.style.zoom_at(617, 234, True)
        self.assertGreaterEqual(self.camera.GetParallelScale(), self.style.MIN_PARALLEL_SCALE)
        for _ in range(200):
            self.style.zoom_at(617, 234, False)
        self.assertLessEqual(self.camera.GetParallelScale(), self.style.MAX_PARALLEL_SCALE)

    def test_pan_moves_position_and_focal_together(self):
        before_position = np.array(self.camera.GetPosition())
        before_focus = np.array(self.camera.GetFocalPoint())
        self.style.pan((200, 250), (240, 260))
        np.testing.assert_allclose(np.array(self.camera.GetPosition()) - before_position,
                                   np.array(self.camera.GetFocalPoint()) - before_focus,
                                   atol=1e-10)


if __name__ == "__main__":
    unittest.main()
