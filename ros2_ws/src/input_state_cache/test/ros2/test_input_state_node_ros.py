import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from vdt_msgs.msg import AltEstimate, InputSnapshot, VisionMarker


def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='input_state_cache',
            executable='input_cache_node',
            parameters=[{'world_frame': 'world'}]),
        launch_testing.actions.ReadyToTest()])


class TestInputCacheNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('isc_test')
        cls.p_ekf = cls.node.create_publisher(Odometry, 'hpad/state_filtered', 10)
        cls.p_mode = cls.node.create_publisher(String, 'ekf/tracking_mode', 10)
        cls.p_odom = cls.node.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
        cls.p_vis = cls.node.create_publisher(VisionMarker, 'vision/marker', 10)
        cls.p_alt = cls.node.create_publisher(AltEstimate, 'alt_estimator/state', 10)
        cls.last = None
        cls.node.create_subscription(
            InputSnapshot, 'input_cache/snapshot',
            lambda m: setattr(cls, 'last', m), 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def feed(self, seconds=1.0, ekf_frame='world', mode='TRACKING'):
        end = time.time() + seconds
        while time.time() < end:
            ekf = Odometry()
            ekf.header.frame_id = ekf_frame
            ekf.pose.pose.position.x = 1.0
            ekf.pose.pose.position.y = 2.0
            odom = Odometry()
            odom.header.frame_id = 'world'
            odom.pose.pose.position.z = 1.5
            vis = VisionMarker()
            vis.marker_visible = True
            vis.pixel_align_error = 0.1
            alt = AltEstimate()
            alt.altitude = 1.5
            self.p_ekf.publish(ekf)
            self.p_mode.publish(String(data=mode))
            self.p_odom.publish(odom)
            self.p_vis.publish(vis)
            self.p_alt.publish(alt)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_1_valid_geometry(self):
        self.feed(1.5)
        s = self.last
        self.assertIsNotNone(s)
        self.assertTrue(s.valid)
        self.assertTrue(s.marker_detected)
        self.assertAlmostEqual(s.d_horiz, math.sqrt(5), places=3)
        self.assertAlmostEqual(s.align_error, s.d_horiz, places=5)
        self.assertAlmostEqual(s.delta_h, 1.5, places=3)
        self.assertAlmostEqual(s.altitude, 1.5, places=3)

    def test_2_wrong_frame_dropped(self):
        self.spin(1.0)
        self.feed(1.0, ekf_frame='map')
        self.assertTrue(math.isnan(self.last.delta_h))
        self.assertTrue(self.last.valid)

    def test_3_expired_mode(self):
        self.feed(1.0, mode='EXPIRED')
        self.assertTrue(math.isnan(self.last.d_horiz))

    def test_4_predicting_mode_valid(self):
        self.feed(1.0, mode='PREDICTING')
        self.assertFalse(math.isnan(self.last.d_horiz))

    def test_5_timeout_when_silent(self):
        self.feed(0.5)
        self.spin(1.0)
        self.assertFalse(self.last.valid)
        self.assertFalse(self.last.marker_detected)

    def test_6_planner_timeout_without_planner(self):
        self.assertTrue(self.last.planner_timeout)