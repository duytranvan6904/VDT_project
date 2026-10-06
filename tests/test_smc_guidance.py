import unittest
import numpy as np
from simulation.control.smc_guidance import SMCGuidance, SMCParams, SMCResult


class TestSMCGuidance(unittest.TestCase):
    def setUp(self):
        self.params = SMCParams(
            v_max=1.50,
            v_final_xy_max=0.35,
            v_descend_fast=0.35,
            v_descend_touch=0.15,
            rswitch_default=0.80,
            final_centering_kp=0.70,
        )
        self.guidance = SMCGuidance(self.params)

    def test_glide_slope_guidance_direction(self):
        """Outside the descent corridor, glide guidance must not descend."""
        drone_pos = np.array([1.0, 1.0, 3.0])
        drone_vel = np.array([0.5, 0.2, -0.2])
        target_pos = np.array([5.0, 2.0, 0.0])  # Target is in +x, +y direction, and below

        res = self.guidance.step(
            drone_pos=drone_pos,
            drone_vel=drone_vel,
            target_pos=target_pos,
            dt=0.05,
        )

        self.assertTrue(res.valid)
        self.assertEqual(res.sub_phase, "GLIDE_SLOPE")
        # Horizontal command should point towards +x and +y
        self.assertGreater(res.velocity_cmd[0], 0.0)  # vx > 0
        self.assertGreater(res.velocity_cmd[1], 0.0)  # vy > 0
        # Hold height until horizontal alignment is inside the pad corridor.
        self.assertEqual(res.velocity_cmd[2], 0.0)

    def test_glide_command_tracks_current_target_error_after_course_drift(self):
        """Glide translation must point to the pad even if internal course is stale."""
        res = self.guidance.step(
            drone_pos=np.array([3.02, 0.56, 2.50]),
            # Deliberately points away from the pad in y; command must use the
            # current EKF position error rather than this integrated course.
            drone_vel=np.array([0.30, -0.20, 0.0]),
            target_pos=np.array([3.00, 1.00, 0.0]),
            dt=0.05,
            rswitch_override=0.80,
        )

        self.assertEqual(res.sub_phase, "GLIDE_SLOPE")
        self.assertLess(res.velocity_cmd[0], 0.0)
        self.assertGreater(res.velocity_cmd[1], 0.0)
        self.assertLess(res.velocity_cmd[2], 0.0)

    def test_glide_includes_target_velocity_feedforward(self):
        """A moving pad's world-frame velocity is added to position pursuit."""
        res = self.guidance.step(
            drone_pos=np.array([0.0, 0.0, 4.0]),
            drone_vel=np.zeros(3),
            target_pos=np.array([0.20, 0.0, 0.0]),
            target_vel=np.array([0.0, 0.50, 0.0]),
            dt=0.05,
            rswitch_override=0.80,
        )

        self.assertGreater(res.velocity_cmd[1], 0.30)
        self.assertLessEqual(np.linalg.norm(res.velocity_cmd), self.params.v_max + 1e-6)

    def test_glide_descends_from_hover_once_inside_landing_corridor(self):
        """A hovering vehicle inside the corridor must get a nonzero descent command."""
        res = self.guidance.step(
            drone_pos=np.array([5.16, 1.89, 4.89]),
            drone_vel=np.array([-0.20, 0.04, 0.0]),
            target_pos=np.array([5.0, 2.0, 0.0]),
            dt=0.05,
            rswitch_override=0.80,
        )

        self.assertEqual(res.sub_phase, "GLIDE_SLOPE")
        self.assertLess(res.velocity_cmd[2], 0.0)
        self.assertGreaterEqual(res.velocity_cmd[2], -self.params.v_descend_fast)
        self.assertLessEqual(np.linalg.norm(res.velocity_cmd), self.params.v_max + 1e-6)

    def test_glide_holds_altitude_outside_horizontal_landing_corridor(self):
        res = self.guidance.step(
            drone_pos=np.array([3.0, 2.0, 4.0]),
            drone_vel=np.array([0.0, 0.0, 0.0]),
            target_pos=np.array([5.0, 2.0, 0.0]),
            dt=0.05,
            rswitch_override=0.80,
        )

        self.assertEqual(res.sub_phase, "GLIDE_SLOPE")
        self.assertEqual(res.velocity_cmd[2], 0.0)

    def test_sub_phase_transition_to_final_descent(self):
        """When Rxy <= Rswitch, guidance must switch to FINAL_DESCENT."""
        target_pos = np.array([5.0, 2.0, 0.0])

        # Drone far away: Rxy ~ 4.1m > 0.8m
        res_far = self.guidance.step(
            drone_pos=np.array([1.0, 1.0, 3.0]),
            drone_vel=np.array([0.5, 0.2, -0.2]),
            target_pos=target_pos,
        )
        self.assertEqual(res_far.sub_phase, "GLIDE_SLOPE")

        # Drone close: Rxy = 0.4m <= 0.8m
        res_close = self.guidance.step(
            drone_pos=np.array([5.2, 2.3, 1.0]),
            drone_vel=np.array([0.1, 0.1, -0.2]),
            target_pos=target_pos,
        )
        self.assertEqual(res_close.sub_phase, "FINAL_DESCENT")

    def test_final_descent_closed_loop_centering(self):
        """In final descent, horizontal velocity commands center onto the pad."""
        target_pos = np.array([5.0, 2.0, 0.0])
        # Drone slightly to the right (+x) and above (+y)
        drone_pos = np.array([5.15, 2.10, 0.60])

        res = self.guidance.step(
            drone_pos=drone_pos,
            drone_vel=np.array([0.0, 0.0, -0.2]),
            target_pos=target_pos,
        )

        self.assertEqual(res.sub_phase, "FINAL_DESCENT")
        # To center from (5.15, 2.10) to (5.0, 2.0), dx = -0.15, dy = -0.10 -> vx < 0, vy < 0
        self.assertLess(res.velocity_cmd[0], 0.0)
        self.assertLess(res.velocity_cmd[1], 0.0)
        self.assertLess(res.velocity_cmd[2], 0.0)

    def test_touchdown_speed_tapering(self):
        """Near ground (altitude < 0.4m), vertical descent speed slows down to v_descend_touch."""
        target_pos = np.array([5.0, 2.0, 0.0])

        # High final descent (z = 0.8m)
        res_high = self.guidance.step(
            drone_pos=np.array([5.05, 2.05, 0.80]),
            drone_vel=np.array([0.0, 0.0, -0.3]),
            target_pos=target_pos,
        )
        self.assertAlmostEqual(res_high.velocity_cmd[2], -self.params.v_descend_fast, places=2)

        # Low final descent (z = 0.25m)
        res_low = self.guidance.step(
            drone_pos=np.array([5.05, 2.05, 0.25]),
            drone_vel=np.array([0.0, 0.0, -0.15]),
            target_pos=target_pos,
        )
        self.assertAlmostEqual(res_low.velocity_cmd[2], -self.params.v_descend_touch, places=2)

    def test_adaptive_rswitch_override(self):
        """An adaptive Rswitch override expands the handover radius."""
        target_pos = np.array([5.0, 2.0, 0.0])
        # Drone at Rxy = 1.2m
        drone_pos = np.array([4.0, 2.0, 1.5])  # dx = 1.0, dy = 0 -> Rxy = 1.0m

        # Default Rswitch = 0.8m -> Rxy (1.0m) > 0.8m -> GLIDE_SLOPE
        res_default = self.guidance.step(
            drone_pos=drone_pos,
            drone_vel=np.array([0.2, 0.0, -0.2]),
            target_pos=target_pos,
            rswitch_override=0.80,
        )
        self.assertEqual(res_default.sub_phase, "GLIDE_SLOPE")

        # Covariance-expanded Rswitch = 1.5m -> Rxy (1.0m) <= 1.5m -> FINAL_DESCENT
        res_adaptive = self.guidance.step(
            drone_pos=drone_pos,
            drone_vel=np.array([0.2, 0.0, -0.2]),
            target_pos=target_pos,
            rswitch_override=1.50,
        )
        self.assertEqual(res_adaptive.sub_phase, "FINAL_DESCENT")

    def test_velocity_limits_enforced(self):
        """Commands must never exceed maximum allowed speed."""
        drone_pos = np.array([0.0, 0.0, 10.0])
        target_pos = np.array([50.0, 50.0, 0.0])

        for _ in range(20):
            res = self.guidance.step(
                drone_pos=drone_pos,
                drone_vel=np.array([1.0, 1.0, -0.5]),
                target_pos=target_pos,
                dt=0.05,
            )
            speed = np.linalg.norm(res.velocity_cmd)
            self.assertLessEqual(speed, self.params.v_max + 1e-4)


if __name__ == '__main__':
    unittest.main()
