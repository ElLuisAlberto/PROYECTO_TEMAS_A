import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from teleop_cooperativo.desktop_console import DesktopState
from teleop_cooperativo.ros_twist_sink import RosTwistSink
from teleop_cooperativo.desktop_qt import finger_link_transform
import numpy as np


class FingerFeedbackTest(unittest.TestCase):
    def test_accepts_three_finite_values_and_rejects_invalid_feedback(self):
        sink = RosTwistSink.__new__(RosTwistSink)
        with patch('teleop_cooperativo.ros_twist_sink.time.monotonic', return_value=10.0):
            sink._on_fingers(SimpleNamespace(data=[0.0, 0.5, 1.0]))
        self.assertEqual(sink._finger_positions, (0.0, 0.5, 1.0))
        self.assertEqual(sink._finger_at, 10.0)
        for values in ([0.1, 0.2], [0.1, float('nan'), 0.2], [0.1, -0.1, 0.2], [0.1, 0.2, 2.0]):
            sink._on_fingers(SimpleNamespace(data=values))
            self.assertIsNone(sink._finger_positions)
            self.assertIsNone(sink._finger_at)

    def test_finger_rotation_moves_tip_in_model_coordinates(self):
        base = np.eye(4)
        origin = np.eye(4)
        zero = finger_link_transform(base, origin, 0.0, distal=True)
        rotated = finger_link_transform(base, origin, math.pi / 2, distal=True)
        np.testing.assert_allclose(zero[:3, 3], [0.044, -0.003, 0.0], atol=1e-8)
        np.testing.assert_allclose(rotated[:3, 3], [0.003, 0.044, 0.0], atol=1e-8)

    def test_console_keeps_fingers_optional_and_invalidates_stale_values(self):
        state = DesktopState()
        message = {'type': 'telemetry', 'source': 'ros_physical_jaco2_fk',
                   'pose': {'position': [0, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1]},
                   'joint_positions': [0.0, math.radians(100), math.radians(100), 0.0, 0.0, 0.0],
                   'finger_positions': [0.0, 0.2, 0.4], 'finger_source': 'simulated', 'pose_age_sec': 0.01}
        state.apply(message)
        self.assertEqual(state.snapshot().finger_positions, (0.0, 0.2, 0.4))
        self.assertEqual(state.snapshot().finger_source, 'simulated')
        message.pop('finger_positions')
        state.apply(message)
        self.assertIsNone(state.snapshot().finger_positions)
        self.assertIsNone(state.snapshot().finger_source)


if __name__ == '__main__':
    unittest.main()
