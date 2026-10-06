"""Regression coverage for moving-target follow and reacquisition handoff."""

import math
from pathlib import Path
import unittest

import numpy as np
import yaml

from simulation.control.tracking_control import (
    TargetMotionGate,
    add_bearing_lead,
    choose_landing_yaw,
    compose_follow_velocity,
    landing_pitch_from_geometry,
    reacquire_candidate_is_valid,
    reacquire_should_resume_search,
    target_bearing_rate,
    target_velocity_feedforward,
    update_search_pitch_command,
    update_tracking_yaw_command,
)
from simulation.control.apf_planner import APFCore, APFParams


class TestFollowVelocityComposition(unittest.TestCase):
    def test_yaw_alignment_does_not_scale_away_target_velocity(self):
        velocity = compose_follow_velocity(
            guidance_xy=[1.0, 0.0],
            target_velocity_ff_xy=[0.5, 0.0],
            alignment_scale=0.15,
            max_speed=1.5,
        )

        np.testing.assert_allclose(velocity, [0.65, 0.0])

    def test_combined_command_remains_inside_follow_speed_limit(self):
        velocity = compose_follow_velocity(
            guidance_xy=[1.4, 0.0],
            target_velocity_ff_xy=[0.5, 0.0],
            alignment_scale=1.0,
            max_speed=1.5,
        )

        self.assertAlmostEqual(float(np.linalg.norm(velocity)), 1.5)

    def test_target_velocity_feedforward_is_retained_at_apf_goal(self):
        result = APFCore(APFParams(goal_threshold=0.30)).compute(
            pos=[0.0, 0.0, 3.0],
            goal=[0.25, 0.0, 3.0],
            obstacles=[],
        )
        ff = target_velocity_feedforward([0.5, 0.0])
        velocity = compose_follow_velocity(
            result.velocity[:2], ff, alignment_scale=0.15, max_speed=1.5,
        )

        self.assertTrue(result.at_goal)
        np.testing.assert_allclose(result.velocity[:2], [0.0, 0.0])
        np.testing.assert_allclose(velocity, [0.5, 0.0])

    def test_stationary_velocity_noise_below_threshold_is_suppressed(self):
        ff = target_velocity_feedforward([0.15, 0.0])

        np.testing.assert_allclose(ff, [0.0, 0.0])

    def test_motion_gate_rejects_single_stationary_target_velocity_spike(self):
        gate = TargetMotionGate(enter_speed=0.35, enter_confirm_frames=3)

        for _ in range(8):
            velocity = gate.update([0.32, 0.0])
            np.testing.assert_allclose(velocity, [0.0, 0.0])
        self.assertFalse(gate.active)

    def test_motion_gate_confirms_real_motion_and_releases_after_settling(self):
        gate = TargetMotionGate(
            enter_speed=0.35,
            exit_speed=0.18,
            enter_confirm_frames=3,
            exit_confirm_frames=4,
        )

        np.testing.assert_allclose(gate.update([0.5, 0.0]), [0.0, 0.0])
        np.testing.assert_allclose(gate.update([0.5, 0.0]), [0.0, 0.0])
        np.testing.assert_allclose(gate.update([0.5, 0.0]), [0.5, 0.0])
        self.assertTrue(gate.active)

        for _ in range(3):
            np.testing.assert_allclose(gate.update([0.0, 0.0]), [0.0, 0.0])
            self.assertTrue(gate.active)
        np.testing.assert_allclose(gate.update([0.0, 0.0]), [0.0, 0.0])
        self.assertFalse(gate.active)


class TestReacquireGate(unittest.TestCase):
    def _candidate(self, **overrides):
        args = dict(
            now=10.20,
            reacquire_started=10.00,
            last_bbox_time=10.10,
            bbox_timeout=0.80,
            bbox_in_roi=True,
            detected=True,
            tracking_mode='TRACKING',
            yaw_rate=0.1,
            max_yaw_rate=math.radians(20.0),
            detection_recent=True,
        )
        args.update(overrides)
        return reacquire_candidate_is_valid(**args)

    def test_requires_a_new_bbox_after_search_sweep_is_stopped(self):
        self.assertFalse(self._candidate(last_bbox_time=9.95))

    def test_accepts_fresh_settled_visual_and_ekf_lock(self):
        self.assertTrue(self._candidate())

    def test_rejects_prediction_only_or_still_rotating_track(self):
        self.assertFalse(self._candidate(tracking_mode='PREDICTING'))
        self.assertFalse(self._candidate(yaw_rate=math.radians(30.0)))

    def test_hold_timeout_never_restarts_sweep_while_marker_is_detected(self):
        self.assertFalse(reacquire_should_resume_search(
            detected=True,
            now=20.0,
            last_detection_time=10.0,
            lost_timeout=2.0,
        ))

    def test_sweep_resumes_only_after_a_real_detection_loss(self):
        self.assertFalse(reacquire_should_resume_search(
            detected=False,
            now=11.9,
            last_detection_time=10.0,
            lost_timeout=2.0,
        ))
        self.assertTrue(reacquire_should_resume_search(
            detected=False,
            now=12.1,
            last_detection_time=10.0,
            lost_timeout=2.0,
        ))


class TestSearchPitchHandoff(unittest.TestCase):
    def test_first_reacquired_bbox_moves_gimbal_toward_target_with_rate_limit(self):
        pitch = update_search_pitch_command(
            current_pitch=math.radians(-30.0),
            pixel_v=360.0,
            image_center_v=240.0,
            focal_y=466.0,
            pixel_gain=0.8,
            trim_gain=0.5,
            dt=1.0 / 30.0,
            pitch_rate_limit=1.5,
            pitch_limits=(math.radians(-60.0), math.radians(-10.0)),
        )

        self.assertLess(pitch, math.radians(-30.0))
        self.assertGreaterEqual(
            pitch,
            math.radians(-30.0) - 1.5 / 30.0 - 1e-9,
        )

    def test_search_pitch_does_not_move_without_a_fresh_bbox(self):
        pitch = update_search_pitch_command(
            current_pitch=math.radians(-30.0),
            pixel_v=None,
            image_center_v=240.0,
            focal_y=466.0,
            pixel_gain=0.8,
            trim_gain=0.5,
            dt=1.0 / 30.0,
            pitch_rate_limit=1.5,
            pitch_limits=(math.radians(-60.0), math.radians(-10.0)),
        )

        self.assertAlmostEqual(pitch, math.radians(-30.0))


class TestYawFeedforward(unittest.TestCase):
    def test_bearing_rate_predicts_a_half_meter_per_second_turn(self):
        rate = target_bearing_rate(3.0, 0.0, 0.0, 0.5)

        self.assertAlmostEqual(rate, 0.5 / 3.0)
        self.assertAlmostEqual(
            add_bearing_lead(0.0, rate, 0.35),
            (0.5 / 3.0) * 0.35,
        )

    def test_stationary_target_has_no_residual_yaw_bias(self):
        rate = target_bearing_rate(3.0, 0.0, 0.0, 0.0)

        self.assertEqual(rate, 0.0)
        self.assertAlmostEqual(add_bearing_lead(0.4, rate, 0.35), 0.4)


class TestTrackingYawCommand(unittest.TestCase):
    def _update(self, **overrides):
        args = dict(
            yaw_command=0.0,
            drone_yaw=0.0,
            pixel_u=None,
            image_center_u=320.0,
            focal_x=466.0,
            pixel_gain=0.92,
            target_delta_xy=(3.0, 0.0),
            target_velocity_xy=(0.0, 0.0),
            tracking_mode='TRACKING',
            target_state_age=0.02,
            target_state_timeout=0.50,
            dt=1.0 / 30.0,
            yaw_rate_limit=0.80,
            feedforward_min_speed=0.30,
            feedforward_max_rate=0.15,
            prediction_horizon=0.25,
        )
        args.update(overrides)
        return update_tracking_yaw_command(**args)

    def test_pixel_right_commands_yaw_toward_right_target(self):
        command = self._update(pixel_u=500.0)

        self.assertLess(command, 0.0)

    def test_short_detector_dropout_keeps_steering_to_fresh_predicted_target(self):
        command = self._update(
            tracking_mode='PREDICTING',
            target_delta_xy=(3.0, 0.5),
        )

        self.assertGreater(command, 0.0)
        self.assertLessEqual(command, 0.80 / 30.0 + 1e-9)

    def test_stale_target_state_cannot_pull_yaw_away(self):
        command = self._update(
            tracking_mode='PREDICTING',
            target_delta_xy=(3.0, 0.5),
            target_state_age=0.60,
        )

        self.assertAlmostEqual(command, 0.0)

    def test_half_meter_per_second_motion_adds_bounded_rate_not_static_bias(self):
        command = self._update(
            pixel_u=320.0,
            target_velocity_xy=(0.0, 0.5),
            dt=0.10,
        )

        self.assertAlmostEqual(command, 0.15 * 0.10, places=6)

    def test_stationary_target_pixel_servo_has_no_pitch_dependent_bias(self):
        command = self._update(
            pixel_u=500.0,
            target_velocity_xy=(0.0, 0.0),
            dt=1.0,
        )
        expected = -0.92 * math.atan((500.0 - 320.0 - 12.0) / 466.0)

        self.assertAlmostEqual(command, expected, places=6)


class TestTrackingConfigInvariants(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config_path = (
            Path(__file__).resolve().parents[1]
            / 'simulation' / 'config' / 'mission_params.yaml'
        )
        with config_path.open(encoding='utf-8') as config_file:
            cls.params = yaml.safe_load(config_file)

    def test_guidance_speed_cap_matches_apf_speed_cap(self):
        apf = self.params['apf_planner']['ros__parameters']
        fsm = self.params['mission_fsm']['ros__parameters']

        self.assertAlmostEqual(apf['v_max'], fsm['follow_speed_limit'])
        self.assertGreaterEqual(fsm['yaw_align_min_speed_scale'], 0.5)
        self.assertGreaterEqual(apf['goal_threshold'], 0.30)

    def test_yaw_rate_limits_agree_across_ibvs_and_offboard(self):
        ibvs = self.params['ibvs_controller']['ros__parameters']
        offboard = self.params['offboard_commander']['ros__parameters']

        self.assertAlmostEqual(ibvs['yaw_rate_limit'], offboard['max_yaw_rate'])
        self.assertGreaterEqual(ibvs['yaw_feedforward_min_target_speed'], 0.30)
        self.assertLessEqual(ibvs['yaw_feedforward_max_rate'], 0.15)

    def test_reacquire_window_is_separate_from_live_landing_freshness(self):
        fsm = self.params['mission_fsm']['ros__parameters']

        self.assertGreater(fsm['reacquire_bbox_timeout'], fsm['follow_bbox_timeout'])
        self.assertLessEqual(
            fsm['landing_tracking_timeout'], fsm['follow_bbox_timeout'],
        )


class TestLandingAimAndYawHold(unittest.TestCase):
    def test_landing_camera_tracks_target_geometry_instead_of_snapping_nadir(self):
        pitch = landing_pitch_from_geometry(1.5, 0.0, 1.0)

        self.assertAlmostEqual(pitch, -math.atan2(1.5, 1.0))
        self.assertGreater(pitch, math.radians(-88.0))

    def test_landing_yaw_latches_last_ibvs_heading_when_marker_is_lost(self):
        _, held_yaw, initialized = choose_landing_yaw(
            tracking_valid=True,
            ibvs_yaw=0.4,
            drone_yaw=0.2,
            held_yaw=0.0,
            hold_initialized=False,
        )
        yaw, held_yaw, initialized = choose_landing_yaw(
            tracking_valid=False,
            ibvs_yaw=-0.7,
            drone_yaw=0.6,
            held_yaw=held_yaw,
            hold_initialized=initialized,
        )

        self.assertAlmostEqual(yaw, 0.4)
        self.assertAlmostEqual(held_yaw, 0.4)
        self.assertTrue(initialized)

    def test_landing_yaw_does_not_chase_pixels_after_approach_latches_heading(self):
        yaw, held_yaw, initialized = choose_landing_yaw(
            tracking_valid=True,
            ibvs_yaw=0.4,
            drone_yaw=0.2,
            held_yaw=0.0,
            hold_initialized=False,
        )
        self.assertAlmostEqual(yaw, 0.4)

        yaw, held_yaw, initialized = choose_landing_yaw(
            tracking_valid=True,
            ibvs_yaw=-1.7,
            drone_yaw=0.3,
            held_yaw=held_yaw,
            hold_initialized=initialized,
        )

        self.assertAlmostEqual(yaw, 0.4)
        self.assertAlmostEqual(held_yaw, 0.4)
        self.assertTrue(initialized)


if __name__ == '__main__':
    unittest.main()
