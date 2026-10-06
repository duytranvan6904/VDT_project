import os
import subprocess
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import rclpy
from vdt_msgs.msg import InputSnapshot

BAG = os.path.join(os.path.dirname(__file__), 'fixtures', 'sample_flight')


def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='input_state_cache',
            executable='input_cache_node',
            parameters=[{'world_frame': 'world'}]),
        launch_testing.actions.ReadyToTest()])


@unittest.skipUnless(os.path.isdir(BAG), 'missing fixtures/sample_flight')
class TestInputCacheRosbag(unittest.TestCase):

    def test_replay(self):
        rclpy.init()
        node = rclpy.create_node('isc_bag')
        got = []
        node.create_subscription(
            InputSnapshot, 'input_cache/snapshot',
            lambda m: got.append((time.time(), m)), 10)
        proc = subprocess.Popen(['ros2', 'bag', 'play', BAG])
        try:
            while proc.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            proc.terminate()
            node.destroy_node()
            rclpy.shutdown()
        self.assertGreater(len(got), 20)
        gaps = [b[0] - a[0] for a, b in zip(got, got[1:])]
        self.assertLess(max(gaps), 0.2)
        self.assertTrue(any(m.valid for _, m in got))
        self.assertTrue(any(m.delta_h == m.delta_h for _, m in got))