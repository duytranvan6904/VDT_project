#!/usr/bin/env python3
"""Keyboard controller for the moving Gazebo H-Pad.

The node sends Gazebo's ``/world/<world>/set_pose`` service through
ros_gz_bridge when available, and falls back to the native ``gz service`` CLI
when the ROS service bridge is missing. It is intentionally a kinematic test controller: the H-Pad
is moved on the ground plane at a configurable speed, so the vision pipeline
can be tested independently of the aircraft controller.

Keys:
  W / S : move +X / -X
  A / D : move +Y / -Y
  SPACE : stop
  R     : reset to the initial pose
  + / - : change speed, limited to 1 m/s by default
  X     : exit
"""

import argparse
import os
import select
import subprocess
import sys
import termios
import time
import tty

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
from simulation.utils.world_geometry import load_hpad_pose_from_sdf

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


def slew_velocity_2d(current, target, max_accel: float, dt: float):
    """Move a planar velocity vector toward its target with a norm acceleration cap."""
    current = (float(current[0]), float(current[1]))
    target = (float(target[0]), float(target[1]))
    dx, dy = target[0] - current[0], target[1] - current[1]
    delta = (dx * dx + dy * dy) ** 0.5
    max_delta = max(0.0, float(max_accel)) * max(0.0, float(dt))
    if delta > max_delta and delta > 1e-9:
        scale = max_delta / delta
        return current[0] + dx * scale, current[1] + dy * scale
    return target


class HpadKeyboardController(Node):
    def __init__(self, args):
        super().__init__('hpad_keyboard_controller')
        self.declare_parameter('world', args.world)
        self.declare_parameter('model', args.model)

        self.world = str(self.get_parameter('world').value)
        self.model = str(self.get_parameter('model').value)
        self.world_sdf = args.world_sdf
        world_pose = load_hpad_pose_from_sdf(self.world_sdf)
        self.partition = args.partition or os.environ.get('GZ_PARTITION', 'vdt_harmonic')
        self.max_speed = min(abs(float(args.max_speed)), 2.0)
        self.speed = min(abs(float(args.speed)), self.max_speed)
        self.accel_limit = max(0.05, float(args.accel_limit))
        self.start_x = float(world_pose[0] if args.x is None else args.x)
        self.start_y = float(world_pose[1] if args.y is None else args.y)
        self.z = float(world_pose[2] if args.z is None else args.z)
        self.x = self.start_x
        self.y = self.start_y
        self.vx = 0.0
        self.vy = 0.0
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.pending = None
        self.pending_target = None
        self.gz_pending = None
        self.gz_target = None
        self.applied_x = self.x
        self.applied_y = self.y
        self.applied_z = self.z
        self.last_update = time.monotonic()
        self.sim_time = None
        self.last_update_sim_time = None
        self.last_request = 0.0
        self.last_log = 0.0
        self.running = True

        self.pose_pub = self.create_publisher(PoseStamped, '/hpad/ground_truth', 10)
        self.cmd_pub = self.create_publisher(Twist, '/hpad/cmd_vel', 10)
        clock_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Clock, '/clock', self.clock_cb, clock_qos)
        self.create_subscription(
            Twist, '/hpad/cmd_vel_cmd', self.command_velocity_cb, 10,
        )
        self.client = self.create_client(
            SetEntityPose, f'/world/{self.world}/set_pose'
        )
        self.bridge_proc = None
        if not args.no_bridge:
            bridge_env = os.environ.copy()
            bridge_env.setdefault('ROS_LOG_DIR', '/tmp/vdt_roslog')
            bridge_env['GZ_PARTITION'] = self.partition
            self.bridge_proc = subprocess.Popen([
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                f'/world/{self.world}/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            ], env=bridge_env)
        self.timer = self.create_timer(0.05, self.update)

    def clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def command_velocity_cb(self, msg: Twist):
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        speed = (vx * vx + vy * vy) ** 0.5
        if speed > self.max_speed and speed > 1e-9:
            scale = self.max_speed / speed
            vx *= scale
            vy *= scale
        self.target_vx, self.target_vy = vx, vy

    def publish_state(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        # Publish only the last pose acknowledged by Gazebo. This prevents
        # RViz from showing a fake motion when the Gazebo request times out.
        pose.pose.position.x = self.applied_x
        pose.pose.position.y = self.applied_y
        pose.pose.position.z = self.applied_z
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        self.cmd_pub.publish(twist)

    def request_pose(self):
        if self.pending is not None or self.gz_pending is not None:
            return
        if (self.x - self.applied_x) ** 2 + (self.y - self.applied_y) ** 2 < 1e-4:
            return
        if self.client.service_is_ready():
            target = (self.x, self.y, self.z)
            request = SetEntityPose.Request()
            request.entity = Entity(name=self.model, type=Entity.MODEL)
            request.pose.position.x = target[0]
            request.pose.position.y = target[1]
            request.pose.position.z = target[2]
            request.pose.orientation.w = 1.0
            self.pending = self.client.call_async(request)
            self.pending_target = target
        else:
            # Keep the controller usable when this ROS service bridge is not
            # installed or did not register. Gazebo UserCommands exposes the
            # equivalent native set_pose service directly.
            target = (self.x, self.y, self.z)
            request_text = (
                f'name: "{self.model}", '
                f'position: {{x: {target[0]:.5f}, y: {target[1]:.5f}, z: {target[2]:.5f}}}, '
                'orientation: {w: 1.0}'
            )
            self.gz_pending = subprocess.Popen([
                'gz', 'service', '-s', f'/world/{self.world}/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', request_text,
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
               env={**os.environ, 'GZ_PARTITION': self.partition})
            self.gz_target = target
        self.last_request = time.monotonic()

    def poll_keyboard(self):
        if not sys.stdin.isatty():
            return
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1).lower()
            if key == 'w':
                self.target_vx, self.target_vy = self.speed, 0.0
            elif key == 's':
                self.target_vx, self.target_vy = -self.speed, 0.0
            elif key == 'a':
                self.target_vx, self.target_vy = 0.0, self.speed
            elif key == 'd':
                self.target_vx, self.target_vy = 0.0, -self.speed
            elif key == ' ':
                self.target_vx = self.target_vy = 0.0
            elif key == 'r':
                self.x, self.y = self.start_x, self.start_y
                self.target_vx = self.target_vy = 0.0
                self.request_pose()
            elif key in ('+', '='):
                self.speed = min(self.max_speed, self.speed + 0.25)
                self.get_logger().info(f'speed={self.speed:.2f} m/s')
            elif key == '-':
                self.speed = max(0.25, self.speed - 0.25)
                self.get_logger().info(f'speed={self.speed:.2f} m/s')
            elif key == 'x':
                self.running = False

    def update(self):
        self.poll_keyboard()
        if not self.running:
            rclpy.shutdown()
            return

        now = time.monotonic()
        wall_dt = min(max(now - self.last_update, 0.0), 0.1)
        self.last_update = now
        if self.sim_time is not None:
            if self.last_update_sim_time is None:
                dt = 0.0
            else:
                dt = min(max(self.sim_time - self.last_update_sim_time, 0.0), 0.1)
            self.last_update_sim_time = self.sim_time
        else:
            # Preserve standalone manual use if Gazebo does not publish /clock.
            dt = wall_dt
        self.vx, self.vy = slew_velocity_2d(
            (self.vx, self.vy),
            (self.target_vx, self.target_vy),
            self.accel_limit,
            dt,
        )
        # Keep the pad inside the 40 m simulation map while leaving enough
        # runway for a moving-pad approach and touchdown at 0.5 m/s.
        self.x = max(-19.0, min(19.0, self.x + self.vx * dt))
        self.y = max(-19.0, min(19.0, self.y + self.vy * dt))

        # Update the pose at 10 Hz so a 0.5 m/s pad advances in small steps
        # (5 cm), which keeps image motion and EKF velocity estimates smooth.
        if now - self.last_request >= 0.10:
            self.request_pose()
        if self.pending is not None and self.pending.done():
            try:
                response = self.pending.result()
                if response.success and self.pending_target is not None:
                    self.applied_x, self.applied_y, self.applied_z = self.pending_target
                else:
                    self.get_logger().warning('Gazebo rejected H-Pad pose update')
            except Exception as exc:
                self.get_logger().warning(f'Pose service failed: {exc}')
            self.pending = None
            self.pending_target = None

        if self.gz_pending is not None and self.gz_pending.poll() is not None:
            stdout, stderr = self.gz_pending.communicate()
            if 'data: true' in stdout.lower() and self.gz_target is not None:
                self.applied_x, self.applied_y, self.applied_z = self.gz_target
            else:
                detail = (stderr or stdout).strip().replace('\n', ' ')
                self.get_logger().warning(
                    f'Native Gazebo set_pose failed: {detail[:180]}'
                )
            self.gz_pending = None
            self.gz_target = None

        self.publish_state()
        if now - self.last_log > 2.0:
            mode = 'ros_service' if self.client.service_is_ready() else 'native_gz_fallback'
            self.get_logger().info(
                f'H-Pad=({self.x:.2f}, {self.y:.2f}) '
                f'applied=({self.applied_x:.2f}, {self.applied_y:.2f}) '
                f'vel=({self.vx:.2f}, {self.vy:.2f}) '
                f'control_mode={mode} partition={self.partition}'
            )
            self.last_log = now

    def close(self):
        if self.bridge_proc is not None and self.bridge_proc.poll() is None:
            self.bridge_proc.terminate()
        if self.gz_pending is not None and self.gz_pending.poll() is None:
            self.gz_pending.terminate()


def parse_args():
    parser = argparse.ArgumentParser(description='Keyboard H-Pad motion controller')
    parser.add_argument('--world', default='obstacle_avoidance')
    parser.add_argument('--model', default='hpad_aruco')
    parser.add_argument(
        '--world-sdf',
        default=os.path.join(REPO_ROOT, 'PX4-Autopilot', 'Tools', 'simulation', 'gz', 'worlds', 'obstacle_avoidance.sdf'),
    )
    parser.add_argument('--x', type=float, default=None)
    parser.add_argument('--y', type=float, default=None)
    parser.add_argument('--z', type=float, default=None)
    parser.add_argument('--speed', type=float, default=0.5)
    parser.add_argument('--max-speed', type=float, default=1.5)
    parser.add_argument('--accel-limit', type=float, default=0.6)
    parser.add_argument('--partition', default=None,
                        help='Gazebo transport partition; defaults to GZ_PARTITION or vdt_harmonic')
    parser.add_argument('--no-bridge', action='store_true',
                        help='Do not start parameter_bridge; use an existing set_pose bridge')
    return parser.parse_args()


def main():
    args = parse_args()
    old_settings = None
    if sys.stdin.isatty():
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    rclpy.init()
    node = HpadKeyboardController(args)
    node.get_logger().info(
        'W/S: X, A/D: Y, SPACE: stop, R: reset, +/-: speed, X: exit'
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
