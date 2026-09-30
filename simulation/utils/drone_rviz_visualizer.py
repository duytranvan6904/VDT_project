#!/usr/bin/env python3
"""
Publishes a 3D Drone Marker & TF Frame into RViz2 so the drone model is visualized.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class DroneRvizVisualizer(Node):
    def __init__(self):
        super().__init__('drone_rviz_visualizer')
        self.marker_pub = self.create_publisher(Marker, '/drone/marker', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.publish_drone_visual)
        self.z = 3.0  # Default altitude matching user's flight

    def publish_drone_visual(self):
        now = self.get_clock().now().to_msg()

        # 1. Broadcast TF for drone frame
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = self.z
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # 2. Publish 3D Drone Visual Marker (Quadcopter Cross)
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = 'world'
        marker.ns = 'drone'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = self.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.scale.z = 1.0
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 1.0
        marker.mesh_resource = "package://gazebo_ros/media/models/quadrotor.dae"

        # Fallback if mesh unavailable: Cube marker
        marker.type = Marker.CUBE
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.15

        self.marker_pub.publish(marker)

def main():
    rclpy.init()
    node = DroneRvizVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
