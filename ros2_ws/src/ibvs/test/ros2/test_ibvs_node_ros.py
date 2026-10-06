import math
import time

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, Float32, Float64, String
from vision_msgs.msg import BoundingBox2D

from ibvs_controller import IBVSController
from ibvs_logic import pixel_pitch_correction

BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def odom_msg(x=0.0, y=0.0, z=0.0, yaw=0.0, vx=0.0, vy=0.0):
    msg = Odometry()
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = float(z)
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    msg.twist.twist.linear.x = float(vx)
    msg.twist.twist.linear.y = float(vy)
    return msg


def bbox_msg(u, v):
    msg = BoundingBox2D()
    msg.center.position.x = float(u)
    msg.center.position.y = float(v)
    return msg


def camera_info_msg(fx, fy, cx, cy):
    msg = CameraInfo()
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    return msg


def set_dt(node, dt):
    node.last_update_time = time.monotonic() - dt


def enter_phase(node, phase, yaw=0.0, detected=True):
    node.odom_cb(odom_msg(yaw=yaw))
    node.phase_cb(String(data=phase))
    node.detected = detected


def give_target(node, x, y, vx=0.0, vy=0.0, mode="TRACKING"):
    node.target_cb(odom_msg(x=x, y=y, vx=vx, vy=vy))
    node.tracking_mode_cb(String(data=mode))


@pytest.fixture
def node():
    instance = IBVSController()
    yield instance
    instance.destroy_node()


def test_initial_state(node):
    assert node.phase == "IDLE"
    assert node.pitch_trim == 0.0
    assert node.tracking_mode == "EXPIRED"
    assert not node.have_odom
    assert node.gimbal_angle_rad == pytest.approx(math.radians(-30.0))


def test_parameters_are_loaded(node):
    assert node.K_pitch == 0.8
    assert node.K_yaw == 0.5
    assert node.fx == node.fy == 466.0
    assert (node.u0, node.v0) == (320.0, 240.0)
    assert node.pitch_trim_limit == pytest.approx(math.radians(20.0))
    assert node.yaw_rate_limit == 0.5


def test_camera_info_overrides_intrinsics(node):
    node.camera_info_cb(camera_info_msg(500.0, 510.0, 300.0, 200.0))
    assert (node.fx, node.fy, node.u0, node.v0) == (500.0, 510.0, 300.0, 200.0)


def test_camera_info_with_zero_k_is_ignored(node):
    node.camera_info_cb(camera_info_msg(0.0, 0.0, 0.0, 0.0))
    assert (node.fx, node.fy, node.u0, node.v0) == (466.0, 466.0, 320.0, 240.0)


def test_gimbal_angle_updates_and_ignores_non_finite(node):
    node.gimbal_angle_cb(Float32(data=-45.0))
    assert node.gimbal_angle_rad == pytest.approx(math.radians(-45.0))
    node.gimbal_angle_cb(Float32(data=float("nan")))
    assert node.gimbal_angle_rad == pytest.approx(math.radians(-45.0))


def test_first_odom_initializes_yaw_cmd_only_once(node):
    node.odom_cb(odom_msg(yaw=0.7))
    assert node.have_odom
    assert node.drone_yaw == pytest.approx(0.7)
    assert node.yaw_cmd == pytest.approx(0.7)
    node.odom_cb(odom_msg(yaw=1.2))
    assert node.drone_yaw == pytest.approx(1.2)
    assert node.yaw_cmd == pytest.approx(0.7)


def test_phase_change_resets_pitch_trim(node):
    node.pitch_trim = -0.1
    node.phase_cb(String(data="FOLLOW"))
    assert node.pitch_trim == 0.0


def test_same_phase_keeps_pitch_trim(node):
    node.phase_cb(String(data="FOLLOW"))
    node.pitch_trim = -0.1
    node.phase_cb(String(data="FOLLOW"))
    assert node.pitch_trim == -0.1


def test_search_to_follow_resets_yaw_cmd(node):
    node.odom_cb(odom_msg(yaw=0.0))
    node.phase_cb(String(data="SEARCH"))
    node.odom_cb(odom_msg(yaw=1.0))
    node.yaw_cmd = -2.0
    node.phase_cb(String(data="FOLLOW"))
    assert node.yaw_cmd == pytest.approx(1.0)


def test_idle_bbox_records_pixel_but_does_not_command(node):
    node.odom_cb(odom_msg(yaw=0.0))
    node.detected = True
    node.bbox_cb(bbox_msg(500.0, 300.0))
    assert node.last_pixel_u == 500.0
    assert node.last_pixel_v == 300.0
    assert node.yaw_cmd == 0.0
    assert node.pitch_trim == 0.0


def test_follow_yaw_turns_right_when_marker_is_right(node):
    enter_phase(node, "FOLLOW")
    set_dt(node, 0.05)
    node.bbox_cb(bbox_msg(node.u0 + 100.0, node.v0))
    assert node.yaw_cmd == pytest.approx(-0.5 * 0.05, abs=1e-3)


def test_follow_yaw_turns_left_when_marker_is_left(node):
    enter_phase(node, "FOLLOW")
    set_dt(node, 0.05)
    node.bbox_cb(bbox_msg(node.u0 - 100.0, node.v0))
    assert node.yaw_cmd == pytest.approx(0.5 * 0.05, abs=1e-3)


def test_follow_yaw_ignores_error_inside_deadband(node):
    enter_phase(node, "FOLLOW")
    set_dt(node, 0.05)
    node.bbox_cb(bbox_msg(node.u0 + 10.0, node.v0))
    assert node.yaw_cmd == pytest.approx(0.0, abs=1e-9)


def test_follow_yaw_step_is_proportional_below_rate_limit(node):
    enter_phase(node, "FOLLOW")
    set_dt(node, 0.05)
    node.bbox_cb(bbox_msg(node.u0 + 13.0, node.v0))
    expected = -0.5 * (1.0 / 466.0) * math.cos(math.radians(-30.0))
    assert node.yaw_cmd == pytest.approx(expected, abs=1e-5)


def test_follow_yaw_step_is_bounded_by_clamped_dt(node):
    enter_phase(node, "FOLLOW")
    set_dt(node, 10.0)
    node.bbox_cb(bbox_msg(node.u0 + 300.0, node.v0))
    assert node.yaw_cmd == pytest.approx(-0.25, abs=1e-3)


def test_follow_yaw_uses_gimbal_angle_for_cosine_scaling(node):
    enter_phase(node, "FOLLOW")
    node.gimbal_angle_cb(Float32(data=-90.0))
    set_dt(node, 0.05)
    node.bbox_cb(bbox_msg(node.u0 + 13.0, node.v0))
    expected = -0.5 * (1.0 / 466.0) * 0.4
    assert node.yaw_cmd == pytest.approx(expected, abs=1e-5)


def test_pixel_is_ignored_when_marker_not_detected(node):
    enter_phase(node, "FOLLOW", detected=False)
    set_dt(node, 0.05)
    node.bbox_cb(bbox_msg(node.u0 + 200.0, node.v0 + 100.0))
    assert node.yaw_cmd == 0.0
    assert node.pitch_trim == 0.0


def test_pixel_only_pitch_trim_first_step(node):
    enter_phase(node, "FOLLOW")
    set_dt(node, 0.05)
    node.bbox_cb(bbox_msg(node.u0, node.v0 + 100.0))
    correction = pixel_pitch_correction(100.0, 0.8, 466.0, 0.5)
    expected = 0.25 * max(correction, -1.5 * 0.05)
    assert node.pitch_trim == pytest.approx(expected, abs=1e-3)
    assert node.pitch_trim < 0.0


def test_pixel_only_pitch_trim_accumulates(node):
    enter_phase(node, "FOLLOW")
    previous = 0.0
    for _ in range(5):
        set_dt(node, 0.05)
        node.bbox_cb(bbox_msg(node.u0, node.v0 + 100.0))
        assert node.pitch_trim < previous
        previous = node.pitch_trim


def test_pitch_trim_is_clamped_to_limit(node):
    enter_phase(node, "FOLLOW")
    node.pitch_trim = -node.pitch_trim_limit
    set_dt(node, 0.05)
    node.bbox_cb(bbox_msg(node.u0, node.v0 + 240.0))
    assert node.pitch_trim >= -node.pitch_trim_limit - 1e-9


def test_pitch_trim_is_positive_when_marker_is_above_center(node):
    enter_phase(node, "FOLLOW")
    set_dt(node, 0.05)
    node.bbox_cb(bbox_msg(node.u0, node.v0 - 100.0))
    assert node.pitch_trim > 0.0


def test_usable_ekf_pulls_trim_to_correction_instead_of_accumulating(node):
    enter_phase(node, "FOLLOW")
    give_target(node, 3.0, 0.0, mode="TRACKING")
    node.pitch_trim = -0.1
    set_dt(node, 0.05)
    node.bbox_cb(bbox_msg(node.u0, node.v0 + 100.0))
    assert node.pitch_trim > -0.1


def test_expired_ekf_falls_back_to_pixel_accumulation(node):
    enter_phase(node, "FOLLOW")
    give_target(node, 3.0, 0.0, mode="EXPIRED")
    node.pitch_trim = -0.1
    set_dt(node, 0.05)
    node.bbox_cb(bbox_msg(node.u0, node.v0 + 100.0))
    assert node.pitch_trim < -0.1


@pytest.mark.parametrize("mode", ["TRACKING", "PREDICTING", "PREDICTING_DEGRADED"])
def test_usable_modes_return_trim_toward_zero_when_pixel_centered(node, mode):
    enter_phase(node, "FOLLOW")
    give_target(node, 3.0, 0.0, mode=mode)
    node.pitch_trim = -0.1
    set_dt(node, 0.05)
    node.bbox_cb(bbox_msg(node.u0, node.v0))
    assert -0.1 < node.pitch_trim < 0.0


def test_yaw_feedforward_follows_tangential_target_motion(node):
    enter_phase(node, "FOLLOW")
    give_target(node, 3.0, 0.0, vy=0.3)
    set_dt(node, 0.05)
    node.bbox_cb(bbox_msg(node.u0, node.v0))
    assert node.yaw_cmd == pytest.approx(0.1 * 0.05, abs=5e-4)


def test_yaw_feedforward_disabled_when_target_is_close(node):
    enter_phase(node, "FOLLOW")
    give_target(node, 0.8, 0.0, vy=0.3)
    set_dt(node, 0.05)
    node.bbox_cb(bbox_msg(node.u0, node.v0))
    assert node.yaw_cmd == pytest.approx(0.0, abs=1e-9)


def test_yaw_feedforward_disabled_when_ekf_unusable(node):
    enter_phase(node, "FOLLOW")
    give_target(node, 3.0, 0.0, vy=0.3, mode="EXPIRED")
    set_dt(node, 0.05)
    node.bbox_cb(bbox_msg(node.u0, node.v0))
    assert node.yaw_cmd == pytest.approx(0.0, abs=1e-9)


def test_lost_marker_turns_toward_ekf_target(node):
    enter_phase(node, "FOLLOW", detected=False)
    give_target(node, 0.0, 5.0)
    set_dt(node, 0.05)
    node.fallback_timer_cb()
    assert node.yaw_cmd == pytest.approx(0.5 * 0.05, abs=1e-3)


def test_lost_marker_uses_predicted_target_position(node):
    enter_phase(node, "FOLLOW", detected=False)
    give_target(node, 5.0, 0.0, vy=-20.0)
    set_dt(node, 0.05)
    node.fallback_timer_cb()
    assert node.yaw_cmd < 0.0


def test_lost_marker_without_usable_ekf_holds_yaw(node):
    enter_phase(node, "FOLLOW", detected=False)
    give_target(node, 0.0, 5.0, mode="EXPIRED")
    set_dt(node, 0.05)
    node.fallback_timer_cb()
    assert node.yaw_cmd == 0.0


def test_lost_marker_with_close_target_holds_yaw(node):
    enter_phase(node, "FOLLOW", detected=False)
    give_target(node, 0.0, 0.2)
    set_dt(node, 0.05)
    node.fallback_timer_cb()
    assert node.yaw_cmd == 0.0


def test_lost_marker_without_ekf_holds_pitch_trim(node):
    enter_phase(node, "FOLLOW", detected=False)
    node.pitch_trim = -0.1
    set_dt(node, 0.05)
    node.fallback_timer_cb()
    assert node.pitch_trim == pytest.approx(-0.1)


def test_approach_behaves_like_follow(node):
    enter_phase(node, "APPROACH")
    set_dt(node, 0.05)
    node.bbox_cb(bbox_msg(node.u0 + 100.0, node.v0 + 100.0))
    assert node.yaw_cmd < 0.0
    assert node.pitch_trim < 0.0


def test_land_zeroes_pitch_trim_and_applies_small_yaw_step(node):
    enter_phase(node, "LAND")
    node.pitch_trim = -0.1
    set_dt(node, 0.05)
    node.bbox_cb(bbox_msg(node.u0 + 100.0, node.v0 + 50.0))
    assert node.pitch_trim == 0.0
    assert node.yaw_cmd == pytest.approx(-0.5 * (100.0 / 466.0) * 0.05, abs=1e-4)


def test_land_without_pixel_snaps_yaw_to_drone_yaw(node):
    enter_phase(node, "LAND", yaw=0.4)
    node.yaw_cmd = 1.0
    node._compute_ibvs(None, None, 0.05)
    assert node.yaw_cmd == pytest.approx(0.4)


def test_search_timer_tracks_drone_yaw_and_zeroes_trim(node):
    enter_phase(node, "SEARCH", yaw=0.7)
    node.pitch_trim = -0.2
    node.yaw_cmd = -1.0
    node.fallback_timer_cb()
    assert node.yaw_cmd == pytest.approx(0.7)
    assert node.pitch_trim == 0.0


def test_idle_timer_tracks_drone_yaw_and_zeroes_trim(node):
    node.odom_cb(odom_msg(yaw=0.3))
    node.pitch_trim = -0.2
    node.yaw_cmd = 2.0
    node.fallback_timer_cb()
    assert node.yaw_cmd == pytest.approx(0.3)
    assert node.pitch_trim == 0.0


class Rig:
    def __init__(self):
        self.executor = SingleThreadedExecutor()
        self.node = IBVSController()
        self.helper = Node("ibvs_test_helper")
        self.executor.add_node(self.node)
        self.executor.add_node(self.helper)
        self.yaws = []
        self.trims = []
        self.helper.create_subscription(Float64, "/ibvs/yaw_cmd", lambda m: self.yaws.append(m.data), 10)
        self.helper.create_subscription(Float32, "/ibvs/pitch_trim_deg", lambda m: self.trims.append(m.data), 10)
        self.bbox_pub = self.helper.create_publisher(BoundingBox2D, "/hpad/bbox", 10)
        self.detected_pub = self.helper.create_publisher(Bool, "/hpad/detected", 10)
        self.mode_pub = self.helper.create_publisher(String, "/ekf/tracking_mode", 10)
        self.target_pub = self.helper.create_publisher(Odometry, "/ekf/target_state", 10)
        self.phase_pub = self.helper.create_publisher(String, "/mission/phase", BEST_EFFORT)
        self.odom_pub = self.helper.create_publisher(Odometry, "/odom", BEST_EFFORT)
        self.gimbal_pub = self.helper.create_publisher(Float32, "/gimbal/target_angle_deg", BEST_EFFORT)
        self.info_pub = self.helper.create_publisher(CameraInfo, "/camera_info", BEST_EFFORT)
        self.spin_for(0.5)

    def spin_until(self, predicate, timeout=3.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return True
        return False

    def spin_for(self, seconds):
        self.spin_until(lambda: False, seconds)

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.helper.destroy_node()


@pytest.fixture
def rig():
    instance = Rig()
    yield instance
    instance.close()


def test_idle_publishes_zero_commands_at_timer_rate(rig):
    rig.yaws.clear()
    rig.trims.clear()
    rig.spin_for(1.0)
    assert len(rig.yaws) >= 15
    assert len(rig.trims) >= 15
    assert all(value == 0.0 for value in rig.yaws)
    assert all(value == 0.0 for value in rig.trims)


def test_idle_yaw_cmd_follows_odom_yaw_over_topic(rig):
    def publish_and_check():
        rig.odom_pub.publish(odom_msg(yaw=1.0))
        return bool(rig.yaws) and abs(rig.yaws[-1] - 1.0) < 1e-6

    assert rig.spin_until(publish_and_check)


def test_camera_info_and_gimbal_topics_update_node(rig):
    def publish_and_check():
        rig.info_pub.publish(camera_info_msg(500.0, 510.0, 300.0, 200.0))
        rig.gimbal_pub.publish(Float32(data=-45.0))
        return rig.node.fx == 500.0 and abs(rig.node.gimbal_angle_rad - math.radians(-45.0)) < 1e-6

    assert rig.spin_until(publish_and_check)
    assert (rig.node.fy, rig.node.u0, rig.node.v0) == (510.0, 300.0, 200.0)


def test_ekf_topics_update_node(rig):
    def publish_and_check():
        rig.mode_pub.publish(String(data="PREDICTING"))
        rig.target_pub.publish(odom_msg(x=1.0, y=2.0, z=3.0, vx=0.1, vy=0.2))
        return rig.node.tracking_mode == "PREDICTING" and rig.node.target_pos is not None

    assert rig.spin_until(publish_and_check)
    assert rig.node.target_pos == [1.0, 2.0, 3.0]
    assert rig.node.target_vel == pytest.approx([0.1, 0.2, 0.0])


def test_follow_pipeline_publishes_corrective_commands(rig):
    rig.yaws.clear()
    rig.trims.clear()
    end = time.monotonic() + 1.2
    while time.monotonic() < end:
        rig.phase_pub.publish(String(data="FOLLOW"))
        rig.odom_pub.publish(odom_msg(yaw=0.0))
        rig.detected_pub.publish(Bool(data=True))
        rig.bbox_pub.publish(bbox_msg(320.0 + 200.0, 240.0 + 100.0))
        rig.spin_for(0.05)
    assert rig.node.phase == "FOLLOW"
    assert rig.yaws[-1] < -0.05
    assert rig.trims[-1] < 0.0
    assert all(math.isfinite(value) for value in rig.yaws + rig.trims)