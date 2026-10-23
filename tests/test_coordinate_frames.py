import math
import unittest

import numpy as np

from simulation.utils.coordinate_frames import rotate_body_vector_to_world


class TestCoordinateFrames(unittest.TestCase):
    def test_identity_quaternion_preserves_velocity(self):
        actual = rotate_body_vector_to_world((0.0, 0.0, 0.0, 1.0), (1.0, -2.0, 0.5))
        np.testing.assert_allclose(actual, (1.0, -2.0, 0.5))

    def test_yaw_quaternion_rotates_body_velocity_into_world(self):
        half_yaw = math.pi / 4.0
        q = (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
        actual = rotate_body_vector_to_world(q, (1.0, 0.0, 0.0))
        np.testing.assert_allclose(actual, (0.0, 1.0, 0.0), atol=1e-7)


if __name__ == '__main__':
    unittest.main()
