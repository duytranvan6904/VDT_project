from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from offboard_manager.msg import PlannerOutput
from rclpy.node import Node
from std_msgs.msg import Float64, String

ACTIVE_PHASES = ('FOLLOW', 'APPROACH')


class PlannerMergeNode(Node):
    def __init__(self) -> None:
        super().__init__('planner_merge')

        defaults = {
            'rate_hz': 20.0,
            'velocity_topic': '/apf/velocity_cmd',
            'apf_yaw_topic': '/apf/yaw_cmd',
            'ibvs_yaw_topic': '/ibvs/yaw_cmd',
            'phase_topic': '/mission/phase',
            'output_topic': 'planner/velocity_setpoint',
            'yaw_source': 'ibvs_apf',
            'upstream_timeout_sec': 0.3,
            'stale_hover_sec': 0.5,
            'yaw_timeout_sec': 0.3,
            'phase_timeout_sec': 1.0,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value

        self.yaw_source = str(g('yaw_source')).strip().lower()
        if self.yaw_source not in ('ibvs_apf', 'ibvs', 'apf', 'hold'):
            self.get_logger().warn(f'Unknown yaw_source "{self.yaw_source}", using ibvs_apf')
            self.yaw_source = 'ibvs_apf'
        self.upstream_timeout = float(g('upstream_timeout_sec'))
        self.stale_hover = float(g('stale_hover_sec'))
        self.yaw_timeout = float(g('yaw_timeout_sec'))
        self.phase_timeout = float(g('phase_timeout_sec'))

        self.phase = 'IDLE'
        self.vel = (0.0, 0.0, 0.0)
        self.apf_yaw = math.nan
        self.ibvs_yaw = math.nan
        self.t_phase = self.t_vel = self.t_apf_yaw = self.t_ibvs_yaw = None

        self.create_subscription(Twist, str(g('velocity_topic')), self.vel_cb, 10)
        self.create_subscription(Float64, str(g('apf_yaw_topic')), self.apf_yaw_cb, 10)
        self.create_subscription(Float64, str(g('ibvs_yaw_topic')), self.ibvs_yaw_cb, 10)
        self.create_subscription(String, str(g('phase_topic')), self.phase_cb, 10)
        self.pub = self.create_publisher(PlannerOutput, str(g('output_topic')), 10)
        self.create_timer(1.0 / float(g('rate_hz')), self.tick)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _age(self, stamp: float | None) -> float:
        return math.inf if stamp is None else self._now() - stamp

    def vel_cb(self, msg: Twist) -> None:
        self.vel = (msg.linear.x, msg.linear.y, msg.linear.z)
        self.t_vel = self._now()

    def apf_yaw_cb(self, msg: Float64) -> None:
        self.apf_yaw = msg.data
        self.t_apf_yaw = self._now()

    def ibvs_yaw_cb(self, msg: Float64) -> None:
        self.ibvs_yaw = msg.data
        self.t_ibvs_yaw = self._now()

    def phase_cb(self, msg: String) -> None:
        self.phase = msg.data
        self.t_phase = self._now()

    def _yaw(self) -> float:
        if self.yaw_source in ('ibvs', 'ibvs_apf'):
            if self._age(self.t_ibvs_yaw) <= self.yaw_timeout and math.isfinite(self.ibvs_yaw):
                return self.ibvs_yaw
        if self.yaw_source in ('apf', 'ibvs_apf'):
            if self._age(self.t_apf_yaw) <= self.yaw_timeout and math.isfinite(self.apf_yaw):
                return self.apf_yaw
        return math.nan

    def tick(self) -> None:
        phase = self.phase if self._age(self.t_phase) <= self.phase_timeout else 'IDLE'
        vx = vy = vz = 0.0
        yaw = math.nan

        if phase in ACTIVE_PHASES:
            age = self._age(self.t_vel)
            if age > self.upstream_timeout + self.stale_hover:
                return
            if age <= self.upstream_timeout:
                vx, vy, vz = self.vel
                yaw = self._yaw()
                if not all(math.isfinite(c) for c in (vx, vy, vz)):
                    vx = vy = vz = 0.0
                    yaw = math.nan

        msg = PlannerOutput()
        if hasattr(msg, 'header'):
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.vx, msg.vy, msg.vz, msg.yaw = float(vx), float(vy), float(vz), float(yaw)
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerMergeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
