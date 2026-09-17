"""Regression tests for FOLLOW stand-off and yaw command stabilization."""

import math
import unittest

import numpy as np

from simulation_maps.apf_planner import APFCore, APFParams, make_follow_goal
from simulation_maps.ibvs_controller import _slew_angle, _wrap_angle


class TestFollowControl(unittest.TestCase):
    def test_follow_goal_keeps_standoff_and_drone_altitude(self):
        drone = np.array([0.0, 0.0, 3.0])
        target = np.array([5.0, 2.0, 0.02])

        goal = make_follow_goal(drone, target, 3.5)

        self.assertAlmostEqual(np.linalg.norm(goal - target), 3.5)
        self.assertAlmostEqual(
            np.linalg.norm(goal[:2] - target[:2]),
            math.sqrt(3.5**2 - (drone[2] - target[2])**2),
        )
        self.assertAlmostEqual(goal[2], drone[2])

        result = APFCore(APFParams(v_max=1.5, k_att=10.0)).compute(
            drone, goal, [],
        )
        self.assertAlmostEqual(result.velocity[2], 0.0)

    def test_yaw_slew_uses_shortest_path_across_pi(self):
        result = _slew_angle(math.radians(179.0), math.radians(-179.0), math.radians(10.0))

        self.assertAlmostEqual(result, math.radians(-179.0), places=6)
        self.assertLess(abs(_wrap_angle(result - math.radians(179.0))), math.radians(10.0))


if __name__ == '__main__':
    unittest.main()
