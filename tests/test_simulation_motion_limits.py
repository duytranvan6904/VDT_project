import math
from pathlib import Path
import unittest

from simulation.core.offboard_commander import _slew_vector_2d
from simulation.utils.hpad_keyboard_controller import slew_velocity_2d
from simulation.utils.flight_telemetry_logger import load_cylinders_from_sdf
from simulation.utils.world_geometry import load_hpad_pose_from_sdf


class TestSimulationMotionLimits(unittest.TestCase):
    def test_offboard_horizontal_acceleration_is_a_vector_norm_limit(self):
        current = (0.0, 0.0)
        target = (1.0, 1.0)

        next_cmd = _slew_vector_2d(current, target, max_delta=0.04)

        self.assertAlmostEqual(math.hypot(*next_cmd), 0.04, places=7)

    def test_hpad_reversals_ramp_without_instant_velocity_flip(self):
        current = (0.5, 0.0)

        next_cmd = slew_velocity_2d(
            current, (-0.5, 0.0), max_accel=0.6, dt=0.05,
        )

        self.assertGreater(next_cmd[0], 0.0)
        self.assertAlmostEqual(math.hypot(next_cmd[0] - current[0], next_cmd[1]), 0.03)

    def test_active_world_is_source_of_hpad_pose_and_obstacles(self):
        repo_root = Path(__file__).resolve().parents[1]
        world_sdf = repo_root / 'PX4-Autopilot/Tools/simulation/gz/worlds/obstacle_avoidance.sdf'

        hpad_pose = load_hpad_pose_from_sdf(str(world_sdf))
        cylinders = load_cylinders_from_sdf(str(world_sdf))

        self.assertAlmostEqual(hpad_pose[0], 3.0)
        self.assertAlmostEqual(hpad_pose[1], 1.0)
        self.assertGreaterEqual(len(cylinders), 1)


if __name__ == '__main__':
    unittest.main()
