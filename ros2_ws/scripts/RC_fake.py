#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from vdt_msgs.msg import RcChannelsRaw, RcFsmInput


class RcFake(Node):
    def __init__(self, kill):
        super().__init__('rc_fake')
        self.kill = kill
        self.pub_raw = self.create_publisher(RcChannelsRaw, 'rc/channels_raw', 10)
        self.pub_fsm = self.create_publisher(RcFsmInput, 'rc/fsm_input', 10)
        self.create_timer(0.02, self.tick)
        self.get_logger().info(f'fake RC started kill={kill}')

    def tick(self):
        ch = [1500] * 16
        ch[3] = 1000
        ch[4] = 2000 if self.kill else 1000
        ch[5] = 2000 if self.kill else 1000
        raw = RcChannelsRaw()
        raw.ch = ch
        raw.valid = True
        raw.failsafe = False
        self.pub_raw.publish(raw)
        self.pub_fsm.publish(RcFsmInput(land_switch=False, kill_switch=self.kill))


def main():
    kill = '--kill' in sys.argv
    rclpy.init()
    node = RcFake(kill)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


main()