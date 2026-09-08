import math
import time

import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from px4_msgs.msg import BatteryStatus, VehicleLocalPosition, VehicleStatus
from std_msgs.msg import Bool


@pytest.mark.rostest
def generate_test_description():
    node = launch_ros.actions.Node(
        package='offboard_safety_monitor',
        executable='safety_monitor_node',
        name='safety_monitor_integration',
        parameters=[{
            'data_freshness_timeout_sec': 0.3,
            'force_land_latched': True,
        }],
        output='screen',
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {'node': node}


class TestSafetyMonitorIntegration:
    @classmethod
    def setup_class(cls):
        rclpy.init()
        cls.node = rclpy.create_node('safety_monitor_integration_test')
        cls.battery_pub = cls.node.create_publisher(BatteryStatus, '/fmu/out/battery_status', 10)
        cls.position_pub = cls.node.create_publisher(VehicleLocalPosition, '/fmu/out/vehicle_local_position', 10)
        cls.status_pub = cls.node.create_publisher(VehicleStatus, '/fmu/out/vehicle_status', 10)
        cls.force_land = None
        cls.force_land_sub = cls.node.create_subscription(
            Bool, 'safety/force_land', cls._force_land_cb, 10)

    @classmethod
    def teardown_class(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _force_land_cb(cls, msg):
        cls.force_land = msg.data

    @classmethod
    def spin_until(cls, predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def publish_inputs(self, remaining):
        battery = BatteryStatus()
        battery.remaining = remaining
        self.battery_pub.publish(battery)

        position = VehicleLocalPosition()
        position.xy_valid = True
        position.z_valid = True
        self.position_pub.publish(position)

        status = VehicleStatus()
        status.nav_state = 14
        status.failsafe = False
        self.status_pub.publish(status)

    def test_invalid_battery_does_not_force_land(self):
        self.publish_inputs(float('nan'))
        assert self.spin_until(lambda: self.force_land is False)

    def test_warning_latches_force_land(self):
        self.publish_inputs(0.2)
        assert self.spin_until(lambda: self.force_land is True)

        self.publish_inputs(0.9)
        time.sleep(0.1)
        rclpy.spin_once(self.node, timeout_sec=0.1)
        assert self.force_land is True

    def test_stale_battery_does_not_clear_latch(self):
        time.sleep(0.4)
        rclpy.spin_once(self.node, timeout_sec=0.1)
        assert self.force_land is True
