import os
import tempfile
import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
import rosbag2_py
from rclpy.serialization import serialize_message
from std_msgs.msg import Bool, Float32, UInt8
from vdt_msgs.msg import InputSnapshot, RcFsmInput

SNAP_TOPIC = '/input_cache/snapshot'
RC_TOPIC = '/rc/fsm_input'
RATE_HZ = 20
BAG_DELAY = 3.0
BASE_NS = 1_000_000_000

GOOD = dict(marker_detected=True, align_error=0.1, altitude=2.0, delta_h=1.0, d_horiz=0.5,
            yaw_rate=0.0, touchdown=False)
LAND = dict(marker_detected=True, align_error=0.15, altitude=0.4, delta_h=0.3, d_horiz=0.1,
            yaw_rate=0.0, touchdown=False)
TOUCHDOWN = dict(marker_detected=True, align_error=0.0, altitude=0.05, delta_h=0.0, d_horiz=0.0,
                 yaw_rate=0.0, touchdown=True)

PHASES = [
    (2.0, GOOD, False),
    (1.0, GOOD, True),
    (2.0, LAND, True),
    (1.0, TOUCHDOWN, True),
]
BAG_DURATION = sum(p[0] for p in PHASES)


def create_topic(writer, index, name, type_name):
    try:
        meta = rosbag2_py.TopicMetadata(name=name, type=type_name, serialization_format='cdr')
    except TypeError:
        meta = rosbag2_py.TopicMetadata(
            id=index, name=name, type=type_name, serialization_format='cdr')
    writer.create_topic(meta)


def build_bag(path):
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))
    create_topic(writer, 0, SNAP_TOPIC, 'vdt_msgs/msg/InputSnapshot')
    create_topic(writer, 1, RC_TOPIC, 'vdt_msgs/msg/RcFsmInput')
    period_ns = int(1e9 / RATE_HZ)
    t_ns = BASE_NS
    for duration, fields, land_switch in PHASES:
        for _ in range(int(duration * RATE_HZ)):
            snap = InputSnapshot()
            snap.valid = True
            snap.planner_timeout = False
            for key, value in fields.items():
                setattr(snap, key, value)
            rc = RcFsmInput()
            rc.land_switch = land_switch
            rc.kill_switch = False
            writer.write(SNAP_TOPIC, serialize_message(snap), t_ns)
            writer.write(RC_TOPIC, serialize_message(rc), t_ns)
            t_ns += period_ns
    del writer


def generate_test_description():
    bag_dir = os.path.join(tempfile.mkdtemp(), 'fsm_bag')
    build_bag(bag_dir)
    fsm = launch_ros.actions.Node(
        package='fsm_state_machine', executable='fsm_node', name='fsm_node', output='screen')
    play = launch.actions.ExecuteProcess(cmd=['ros2', 'bag', 'play', bag_dir], output='screen')
    description = launch.LaunchDescription([
        fsm,
        launch.actions.TimerAction(period=BAG_DELAY, actions=[play]),
        launch_testing.actions.ReadyToTest(),
    ])
    return description, {'fsm': fsm}


def dedupe(values):
    out = []
    for v in values:
        if not out or out[-1] != v:
            out.append(v)
    return out


class TestFsmRosbag(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('fsm_rosbag_probe')
        cls.states = []
        cls.modes = []
        cls.gains = []
        cls.descents = []
        cls.disarms = []
        cls.node.create_subscription(UInt8, '/fsm/state', lambda m: cls.states.append(m.data), 50)
        cls.node.create_subscription(UInt8, '/planner/mode', lambda m: cls.modes.append(m.data), 50)
        cls.node.create_subscription(
            Float32, '/planner/apf_gain', lambda m: cls.gains.append(m.data), 50)
        cls.node.create_subscription(
            Float32, '/cmd/vertical_descent_rate', lambda m: cls.descents.append(m.data), 50)
        cls.node.create_subscription(
            Bool, '/cmd/disarm_request', lambda m: cls.disarms.append(m.data), 50)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def collect(self):
        end = time.time() + BAG_DELAY + BAG_DURATION + 4.0
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_state_sequence_and_commands(self, proc_output=None):
        self.collect()
        self.assertGreater(len(self.states), 0)
        self.assertEqual(dedupe(self.states), [0, 1, 2, 3, 4])
        self.assertEqual(dedupe(self.modes), [0, 1, 2, 3])
        self.assertIn(0.5, [round(g, 3) for g in self.gains])
        self.assertIn(1.0, [round(g, 3) for g in self.gains])
        self.assertIn(0.0, [round(g, 3) for g in self.gains])
        rounded = [round(d, 3) for d in self.descents]
        self.assertIn(0.3, rounded)
        self.assertIn(0.4, rounded)
        self.assertTrue(len(self.disarms) > 0 and all(self.disarms))
        self.assertEqual(self.states[-1], 4)

    def test_states_never_go_backwards(self):
        self.collect()
        states = dedupe(self.states)
        self.assertEqual(states, sorted(states))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])