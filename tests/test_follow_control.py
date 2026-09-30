"""Regression tests for FOLLOW stand-off and yaw command stabilization."""

import math
import unittest

import numpy as np

from simulation.control.apf_planner import APFCore, APFParams, make_follow_goal
from simulation.control.ibvs_controller import _slew_angle, _wrap_angle


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

    def test_follow_goal_decouples_standoff_from_altitude_fluctuations(self):
        target = np.array([5.0, 2.0, 0.0])
        drone_low = np.array([0.0, 0.0, 2.90])
        drone_high = np.array([0.0, 0.0, 3.10])

        goal_low = make_follow_goal(drone_low, target, 3.5, hold_altitude=True, cruise_altitude=3.0)
        goal_high = make_follow_goal(drone_high, target, 3.5, hold_altitude=True, cruise_altitude=3.0)

        standoff_low = float(np.linalg.norm(goal_low[:2] - target[:2]))
        standoff_high = float(np.linalg.norm(goal_high[:2] - target[:2]))

        # Horizontal standoff must remain identical despite altitude variations
        self.assertAlmostEqual(standoff_low, standoff_high, places=5)
        self.assertAlmostEqual(standoff_low, math.sqrt(3.5**2 - 3.0**2), places=5)
        self.assertAlmostEqual(goal_low[2], 3.0)
        self.assertAlmostEqual(goal_high[2], 3.0)


if __name__ == '__main__':
    unittest.main()
