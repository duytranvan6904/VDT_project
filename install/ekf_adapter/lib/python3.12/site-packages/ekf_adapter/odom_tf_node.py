import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class OdomTfNode(Node):
    def __init__(self) -> None:
        super().__init__("odom_tf_node")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.broadcaster = TransformBroadcaster(self)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.odom_callback, qos)

    def odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        q = pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-9:
            self.get_logger().warning("odometry quaternion has zero norm", throttle_duration_sec=2.0)
            return
        transform = TransformStamped()
        if msg.header.stamp.sec > 0 or msg.header.stamp.nanosec > 0:
            transform.header.stamp = msg.header.stamp
        else:
            transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation.x = q.x / norm
        transform.transform.rotation.y = q.y / norm
        transform.transform.rotation.z = q.z / norm
        transform.transform.rotation.w = q.w / norm
        self.broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = OdomTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
