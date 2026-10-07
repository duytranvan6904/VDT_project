#!/usr/bin/env python3
"""Tests for safety mechanisms, manual override handling, and Mission FSM land commands."""

import unittest
from unittest.mock import MagicMock
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry

from simulation.core.mission_fsm_node import MissionFSMNode, MissionPhase
from simulation.core.offboard_commander import OffboardCommander


class TestMissionFSMAndSafety(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.fsm = MissionFSMNode()
        self.offboard = OffboardCommander()

    def tearDown(self):
        self.fsm.destroy_node()
        self.offboard.destroy_node()

    def test_land_command_in_follow_transitions_to_approach_without_attribute_error(self):
        """Receiving land_command in FOLLOW with valid tracking enters APPROACH safely."""
        self.fsm.phase = MissionPhase.FOLLOW
        self.fsm.has_odom = True
        self.fsm.drone_pos = np.array([0.0, 0.0, 3.0])
        self.fsm.drone_yaw = 0.5
        self.fsm.has_target = True
        self.fsm.target_pos = np.array([2.0, 0.0, 0.0])
        self.fsm.tracking_mode = 'TRACKING'
        self.fsm.has_bbox = True
        self.fsm.last_bbox_time = 1e9  # far in future so it is recent
        self.fsm.last_odom_time = 1e9
        self.fsm.last_target_time = 1e9
        self.fsm.detected = True

        msg = Bool(data=True)
        self.fsm.land_cmd_cb(msg)
        self.fsm._handle_follow()

        self.assertEqual(self.fsm.phase, MissionPhase.APPROACH)
        self.assertFalse(self.fsm.landing_hold_initialized)

    def test_manual_override_forces_fsm_to_idle(self):
        """When manual override is received from pilot, FSM immediately reverts to IDLE."""
        self.fsm.phase = MissionPhase.APPROACH
        self.fsm.land_requested = True
        self.fsm.landing_glide_active = True

        override_msg = Bool(data=True)
        self.fsm.manual_override_cb(override_msg)

        self.assertEqual(self.fsm.phase, MissionPhase.IDLE)
        self.assertFalse(self.fsm.land_requested)
        self.assertFalse(self.fsm.landing_glide_active)

    def test_offboard_commander_detects_pilot_takeover(self):
        """When PX4 switches away from OFFBOARD (mode 6), offboard disengages immediately."""
        # Case 1: Initial handshake before confirmation does NOT trigger false manual override
        self.offboard.offboard_engaged = True
        self.offboard.offboard_confirmed = False
        self.offboard.manual_override = False
        self.offboard.px4_current_main_mode = 4  # Still in takeoff/auto mode during switch
        self.offboard.heartbeat_cb()
        self.assertTrue(self.offboard.offboard_engaged)
        self.assertFalse(self.offboard.manual_override)

        # Case 2: Once OFFBOARD is confirmed, pilot switching to AUTO_LAND (4) or POSCTL (3) triggers immediate override
        self.offboard.offboard_confirmed = True
        self.offboard.heartbeat_cb()

        self.assertFalse(self.offboard.offboard_engaged)
        self.assertTrue(self.offboard.manual_override)

        # Setpoints must not be published while manual override is active
        mock_mav = MagicMock()
        self.offboard.mav_conn = mock_mav
        self.offboard.publish_setpoint()
        mock_mav.mav.set_position_target_local_ned_send.assert_not_called()

    def test_search_mode_holds_steady_yaw_and_sweeps_pitch_when_yaw_rate_is_zero(self):
        """When search_yaw_rate == 0, drone holds yaw steady and sweeps pitch without spinning."""
        from simulation.control.ibvs_controller import IBVSController
        ibvs = IBVSController()
        ibvs.have_odom = True
        ibvs.drone_yaw = 0.75
        ibvs.phase = 'SEARCH'
        ibvs.search_entry_hold_active = False
        ibvs.detected = False
        ibvs.search_hold_active = False
        ibvs.search_yaw_rate = 0.0
        ibvs.search_pitch_sweep_enable = True

        initial_pitch = ibvs.gimbal_pitch
        ibvs.fallback_timer_cb()

        # Yaw must remain held steady at drone heading (no body spinning)
        self.assertAlmostEqual(ibvs.yaw_cmd, 0.75)
        # Gimbal pitch must move according to sweep
        self.assertNotEqual(ibvs.gimbal_pitch, initial_pitch)
        ibvs.destroy_node()


if __name__ == '__main__':
    unittest.main()
