from __future__ import annotations

import math
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, String
from visualization_msgs.msg import MarkerArray

from .apf_core import APFCore, APFParams, make_follow_goal
from .iapf_core import IAPFCore, IAPFParams
from .markers import build_markers
from .obstacles import (
    cloud_to_xyz,
    cylinder_nearest_point,
    load_cylinder_obstacles_from_sdf,
    select_obstacle_points,
    voxel_downsample,
)


class APFPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('apf_planner')

        defaults = {
            'planner_type': 'apf',
            'obstacle_source': 'pointcloud',
            'world_sdf': '',
            'world_frame': 'world',
            'odom_topic': '/odom',
            'target_topic': '/hpad/state_filtered',
            'mode_topic': '/ekf/tracking_mode',
            'phase_topic': '/mission/phase',
            'cloud_topic': '/map_generator/global_cloud',
            'allowed_tracking_modes': ['TRACKING', 'PREDICTING'],
            'data_timeout_sec': 0.5,
            'rate_hz': 30.0,
            'd0': 2.0,
            'v_max': 1.2,
            'd_slow': 1.5,
            'k_att': 10.0,
            'k_rep': 250.0,
            'k_rep_approach': 125.0,
            'goal_threshold': 0.20,
            'follow_distance': 3.5,
            'hold_follow_altitude': True,
            'target_altitude': 3.0,
            'k_z': 0.6,
            'vz_max': 0.5,
            'target_lead_time': 0.25,
            'max_target_lead': 0.75,
            'target_velocity_alpha': 0.25,
            'target_velocity_deadband': 0.08,
            'cloud_voxel_size': 0.3,
            'cloud_query_radius': 0.0,
            'cloud_cluster_radius': 1.2,
            'max_cloud_points': 8,
            'iapf_f_enter': 0.10,
            'iapf_f_exit': 0.30,
            'iapf_k_tan': 1.0,
            'iapf_n_tangent': 12,
            'iapf_n_pred': 3,
            'iapf_w_goal': 1.0,
            'iapf_w_clear': 1.0,
            'iapf_w_prev': 0.60,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value

        self.planner_type = str(g('planner_type')).strip().lower()
        d0, v_max = float(g('d0')), float(g('v_max'))
        if self.planner_type == 'iapf':
            self.core = IAPFCore(IAPFParams(
                k_att=float(g('k_att')), k_rep=float(g('k_rep')), d0=d0, v_max=v_max,
                goal_tol=float(g('goal_threshold')),
                f_enter=float(g('iapf_f_enter')), f_exit=float(g('iapf_f_exit')),
                k_tan=float(g('iapf_k_tan')), n_tangent=int(g('iapf_n_tangent')),
                n_pred=int(g('iapf_n_pred')), w_goal=float(g('iapf_w_goal')),
                w_clear=float(g('iapf_w_clear')), w_prev=float(g('iapf_w_prev')),
                vz_max=float(g('vz_max'))))
        else:
            self.planner_type = 'apf'
            self.core = APFCore(APFParams(
                d0=d0, v_max=v_max, d_slow=float(g('d_slow')),
                k_att=float(g('k_att')), k_rep=float(g('k_rep')),
                goal_threshold=float(g('goal_threshold')),
                k_z=float(g('k_z')), vz_max=float(g('vz_max'))))
        self.get_logger().info(f'Planner: {self.planner_type}')

        self.world_frame = str(g('world_frame'))
        self.allowed_modes = set(g('allowed_tracking_modes'))
        self.data_timeout = float(g('data_timeout_sec'))
        self.k_rep_approach = float(g('k_rep_approach'))
        self.follow_distance = float(g('follow_distance'))
        self.hold_alt = bool(g('hold_follow_altitude'))
        self.target_alt = float(g('target_altitude'))
        self.lead_time = float(g('target_lead_time'))
        self.max_lead = float(g('max_target_lead'))
        self.vel_alpha = float(np.clip(g('target_velocity_alpha'), 0.0, 1.0))
        self.vel_deadband = float(g('target_velocity_deadband'))
        self.voxel = float(g('cloud_voxel_size'))
        self.cluster_r = float(g('cloud_cluster_radius'))
        self.max_pts = int(g('max_cloud_points'))
        self.query_r = float(g('cloud_query_radius')) or 2.0 * d0

        self.drone_pos = np.zeros(3)
        self.goal_pos: Optional[np.ndarray] = None
        self.target_vel = np.zeros(3)
        self.phase = 'IDLE'
        self.tracking_mode = 'LOST'
        self.t_odom = None
        self.t_target = None
        self.cloud = np.empty((0, 3))
        self.cylinders = []

        source = str(g('obstacle_source')).strip().lower()
        if source in ('sdf', 'both'):
            self.cylinders = load_cylinder_obstacles_from_sdf(str(g('world_sdf')))
            self.get_logger().info(f'Loaded {len(self.cylinders)} SDF cylinders')

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Odometry, str(g('odom_topic')), self.odom_cb, best_effort)
        self.create_subscription(Odometry, str(g('target_topic')), self.target_cb, 10)
        self.create_subscription(String, str(g('phase_topic')), self.phase_cb, 10)
        self.create_subscription(String, str(g('mode_topic')), self.mode_cb, 10)
        if source in ('pointcloud', 'both'):
            self.create_subscription(PointCloud2, str(g('cloud_topic')), self.cloud_cb, 1)

        self.vel_pub = self.create_publisher(Twist, '/apf/velocity_cmd', 10)
        self.yaw_pub = self.create_publisher(Float64, '/apf/yaw_cmd', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/apf/force_markers', 10)
        self.create_timer(1.0 / float(g('rate_hz')), self.tick)

    def _frame_ok(self, msg) -> bool:
        fid = msg.header.frame_id
        if fid and fid != self.world_frame:
            self.get_logger().warn(
                f'Ignoring frame "{fid}" (expected "{self.world_frame}")',
                throttle_duration_sec=5.0)
            return False
        return True

    def odom_cb(self, msg: Odometry) -> None:
        if not self._frame_ok(msg):
            return
        p = msg.pose.pose.position
        pos = np.array([p.x, p.y, p.z])
        if not np.isfinite(pos).all():
            return
        self.drone_pos = pos
        self.t_odom = self.get_clock().now()

    def target_cb(self, msg: Odometry) -> None:
        if not self._frame_ok(msg):
            return
        p = msg.pose.pose.position
        pos = np.array([p.x, p.y, p.z])
        if not np.isfinite(pos).all():
            return
        self.goal_pos = pos
        self.t_target = self.get_clock().now()
        v = msg.twist.twist.linear
        raw = np.array([v.x, v.y, v.z])
        if not np.isfinite(raw).all():
            raw = np.zeros(3)
        if float(np.linalg.norm(raw[:2])) < self.vel_deadband:
            raw[:2] = 0.0
        if abs(raw[2]) < self.vel_deadband:
            raw[2] = 0.0
        a = self.vel_alpha
        self.target_vel = a * raw + (1.0 - a) * self.target_vel
        if float(np.linalg.norm(self.target_vel[:2])) < 0.08:
            self.target_vel[:2] = 0.0
        if abs(self.target_vel[2]) < 0.08:
            self.target_vel[2] = 0.0

    def phase_cb(self, msg: String) -> None:
        if msg.data != self.phase and hasattr(self.core, 'reset'):
            self.core.reset()
        self.phase = msg.data

    def mode_cb(self, msg: String) -> None:
        self.tracking_mode = msg.data

    def cloud_cb(self, msg: PointCloud2) -> None:
        if not self._frame_ok(msg):
            return
        self.cloud = voxel_downsample(cloud_to_xyz(msg), self.voxel)

    def _fresh(self, stamp) -> bool:
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9 <= self.data_timeout

    def _goal(self) -> Optional[np.ndarray]:
        if self.goal_pos is None:
            return None
        if self.phase != 'FOLLOW':
            return self.goal_pos.copy()
        lead = np.zeros(3)
        if float(np.linalg.norm(self.target_vel[:2])) >= 0.18:
            lead = self.target_vel * self.lead_time
            n = float(np.linalg.norm(lead[:2]))
            if n > self.max_lead and n > 1e-9:
                lead[:2] *= self.max_lead / n
            lead[2] = float(np.clip(lead[2], -self.max_lead, self.max_lead))
        return make_follow_goal(
            self.drone_pos, self.goal_pos + lead, self.follow_distance,
            hold_altitude=self.hold_alt, cruise_altitude=self.target_alt)

    def _obstacles(self) -> List[tuple]:
        pts = [tuple(cylinder_nearest_point(self.drone_pos, *c)) for c in self.cylinders]
        pts += select_obstacle_points(
            self.cloud, self.drone_pos, self.query_r, self.cluster_r, self.max_pts)
        return pts

    def tick(self) -> None:
        goal = self._goal()
        active = (
            self._fresh(self.t_odom)
            and self._fresh(self.t_target)
            and goal is not None
            and self.phase in ('FOLLOW', 'APPROACH')
            and self.tracking_mode in self.allowed_modes
        )
        have_odom = self.t_odom is not None
        stamp = self.get_clock().now().to_msg()

        if not active:
            self.vel_pub.publish(Twist())
            self.marker_pub.publish(build_markers(
                stamp, self.world_frame, self.drone_pos if have_odom else None,
                goal, None, self.core.params, self.cylinders, []))
            return

        obstacles = self._obstacles()
        k_rep = self.k_rep_approach if self.phase == 'APPROACH' else None
        result = self.core.compute(
            self.drone_pos, goal, obstacles,
            k_rep_override=k_rep, target_pos=self.goal_pos)

        vx, vy, vz = (float(c) for c in result.velocity)
        if self.phase == 'FOLLOW' and not result.at_goal:
            if float(np.linalg.norm(self.target_vel[:2])) >= 0.20:
                vx += float(self.target_vel[0])
                vy += float(self.target_vel[1])
                h = math.hypot(vx, vy)
                v_max = float(self.core.params.v_max)
                if h > v_max:
                    vx *= v_max / h
                    vy *= v_max / h

        if not all(math.isfinite(c) for c in (vx, vy, vz, result.yaw_cmd)):
            self.vel_pub.publish(Twist())
            return

        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = vx, vy, vz
        self.vel_pub.publish(cmd)
        self.yaw_pub.publish(Float64(data=float(result.yaw_cmd)))
        self.marker_pub.publish(build_markers(
            stamp, self.world_frame, self.drone_pos, goal, result,
            self.core.params, self.cylinders, obstacles))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = APFPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
