#!/usr/bin/env python3
"""
APF Simulation Launcher - Inspired by Fast-Tracker & PX4-Avoidance
==================================================================
Chạy hệ thống mô phỏng chuẩn xác dựa theo kiến trúc Fast-Tracker & PX4-Avoidance:
1. Đồng bộ Map 3D ngẫu nhiên giữa Gazebo Sim và PointCloud2 toàn cục (/map_generator/global_cloud)
2. Bridge dữ liệu PointCloud2 cảm biến (/depth_camera/points) chuẩn xác từ Gazebo Sim về ROS 2
3. Bridge dữ liệu vị trí & Odometry thực tế của Drone từ Gazebo (/model/x500_depth_0/odometry_with_covariance) -> /odom & TF (world -> base_link)
4. Mở RViz2 tự động hiển thị Map 3D, Drone 3D và PointCloud Cảm biến theo thời gian thực
"""

import os
import sys
import math
import time
import random
import struct
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

# ── Tham số Map ─────────────────────────────────────────────────────────────
NUM_OBS = 15
MAP_SIZE = 20.0
MAX_HEIGHT = 4.0
MODEL_NAME = "x500_depth_0"

def generate_obstacles():
    half = MAP_SIZE / 2.0
    obs = []
    for _ in range(NUM_OBS):
        cx, cy = random.uniform(-half, half), random.uniform(-half, half)
        if math.sqrt(cx**2 + cy**2) < 3.0:
            continue
        obs.append({
            'cx': cx, 'cy': cy,
            'r': random.uniform(0.35, 0.65),
            'h': random.uniform(2.5, MAX_HEIGHT),
            'color': (random.uniform(0.2, 0.9), random.uniform(0.2, 0.9), random.uniform(0.2, 0.9))
        })
    return obs

def write_sdf(obstacles, path):
    models = ""
    for i, o in enumerate(obstacles, 1):
        cx, cy, r, h = o['cx'], o['cy'], o['r'], o['h']
        cr, cg, cb = o['color']
        models += f"""
    <model name="cyl_{i}"><static>true</static>
      <pose>{cx:.2f} {cy:.2f} {h/2:.2f} 0 0 0</pose>
      <link name="l"><collision name="c"><geometry><cylinder>
        <radius>{r:.2f}</radius><length>{h:.2f}</length>
      </cylinder></geometry></collision>
      <visual name="v"><geometry><cylinder>
        <radius>{r:.2f}</radius><length>{h:.2f}</length>
      </cylinder></geometry>
      <material><ambient>{cr:.2f} {cg:.2f} {cb:.2f} 1</ambient>
        <diffuse>{cr:.2f} {cg:.2f} {cb:.2f} 1</diffuse></material>
      </visual></link>
    </model>"""

    sdf = f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="obstacle_avoidance">
    <physics type="ode"><max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate></physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene><grid>true</grid><ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background><shadows>false</shadows></scene>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction></light>
    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal>
        <size>200 200</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal>
        <size>200 200</size></plane></geometry>
        <material><ambient>0.7 0.7 0.7 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse></material></visual>
    </link></model>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
{models}
  </world>
</sdf>
"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(sdf)

def build_pointcloud(obstacles, res=0.15):
    pts = []
    for o in obstacles:
        cx, cy, r, h = o['cx'], o['cy'], o['r'], o['h']
        z = 0.0
        while z <= h:
            n = max(int(2 * math.pi * r / res), 8)
            for i in range(n):
                th = i * 2 * math.pi / n
                pts.append((cx + r * math.cos(th), cy + r * math.sin(th), z))
            z += res
    return pts

class FastTrackerStyleNode(Node):
    def __init__(self, obstacles):
        super().__init__('fast_tracker_style_node')

        # Best Effort QoS cho sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.cloud_pub = self.create_publisher(PointCloud2, '/map_generator/global_cloud', 10)
        self.drone_marker_pub = self.create_publisher(Marker, '/drone/marker', 10)

        # TF Broadcasters
        self.static_tf = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Broadcast world -> map static frame
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = 'world'
        st.child_frame_id = 'map'
        st.transform.rotation.w = 1.0
        self.static_tf.sendTransform(st)

        # Drone State (Cập nhật trực tiếp từ Gazebo Odometry)
        self.drone_pos = [0.0, 0.0, 0.0]
        self.drone_quat = [0.0, 0.0, 0.0, 1.0]
        self.has_odom = False
        self.has_sensor_cloud = False

        # Subscribe Odometry từ Gazebo
        odom_topic = f"/model/{MODEL_NAME}/odometry_with_covariance"
        self.create_subscription(Odometry, odom_topic, self.odom_callback, sensor_qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, sensor_qos)

        # Subscribe PointCloud2 từ Depth Camera của Drone
        self.create_subscription(PointCloud2, '/depth_camera/points', self.sensor_cloud_callback, sensor_qos)

        # Build pointcloud cho bản đồ toàn cục
        self.pc_pts = build_pointcloud(obstacles)

        # Timers
        self.create_timer(1.0, self.publish_global_map)
        self.create_timer(0.05, self.publish_drone_state)
        self.create_timer(3.0, self.log_status)

        self.get_logger().info("Fast-Tracker & PX4-Avoidance Simulation Node Active!")

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.drone_pos = [p.x, p.y, p.z]
        self.drone_quat = [q.x, q.y, q.z, q.w]
        self.has_odom = True

    def sensor_cloud_callback(self, msg: PointCloud2):
        self.has_sensor_cloud = True

    def publish_global_map(self):
        h = Header(stamp=self.get_clock().now().to_msg(), frame_id='world')
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        data = b"".join(struct.pack('fff', *p) for p in self.pc_pts)
        msg = PointCloud2(header=h, height=1, width=len(self.pc_pts),
                          fields=fields, is_bigendian=False, point_step=12,
                          row_step=12*len(self.pc_pts), is_dense=True, data=data)
        self.cloud_pub.publish(msg)

    def publish_drone_state(self):
        now = self.get_clock().now().to_msg()

        # 1. Phát TF Frame (world -> base_link)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.drone_pos[0]
        t.transform.translation.y = self.drone_pos[1]
        t.transform.translation.z = self.drone_pos[2]
        t.transform.rotation.x = self.drone_quat[0]
        t.transform.rotation.y = self.drone_quat[1]
        t.transform.rotation.z = self.drone_quat[2]
        t.transform.rotation.w = self.drone_quat[3]
        self.tf_broadcaster.sendTransform(t)

        # 2. Phát 3D Drone Visual Marker trong RViz2
        m = Marker()
        m.header.stamp = now
        m.header.frame_id = 'world'
        m.ns = 'drone'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = self.drone_pos[0]
        m.pose.position.y = self.drone_pos[1]
        m.pose.position.z = self.drone_pos[2]
        m.pose.orientation.x = self.drone_quat[0]
        m.pose.orientation.y = self.drone_quat[1]
        m.pose.orientation.z = self.drone_quat[2]
        m.pose.orientation.w = self.drone_quat[3]
        m.scale.x = 0.45
        m.scale.y = 0.45
        m.scale.z = 0.15
        m.color.r = 0.0
        m.color.g = 0.8
        m.color.b = 1.0
        m.color.a = 0.9
        self.drone_marker_pub.publish(m)

    def log_status(self):
        odom_str = "CONNECTED" if self.has_odom else "WAITING"
        cloud_str = "RECEIVING" if self.has_sensor_cloud else "WAITING"
        pos_str = f"({self.drone_pos[0]:.2f}, {self.drone_pos[1]:.2f}, {self.drone_pos[2]:.2f})"
        self.get_logger().info(f"[STATUS] Gazebo Odom: [{odom_str}] | Camera PointCloud2: [{cloud_str}] | Drone Pos: {pos_str}")

def main():
    print("=" * 70)
    print("  SIMULATION LAUNCHER (Fast-Tracker & PX4-Avoidance Architecture)")
    print("=" * 70)

    # 1. Sinh bản đồ chướng ngại vật ngẫu nhiên
    print("[1/4] Generating Obstacle World...")
    obstacles = generate_obstacles()
    for p in [
        "/home/duy/VDT_project/simulation_maps/gazebo_worlds/obstacle_avoidance.sdf",
        "/home/duy/VDT_project/PX4-Autopilot/Tools/simulation/gz/worlds/obstacle_avoidance.sdf",
    ]:
        write_sdf(obstacles, p)
    print(f"      -> Map generated with {len(obstacles)} obstacles.")

    # 2. Khởi chạy ros_gz_bridge với cú pháp Direction chuẩn ([ = Gazebo -> ROS 2)
    print("[2/4] Starting Gazebo Sim -> ROS 2 Bridge...")
    odom_gz = f"/model/{MODEL_NAME}/odometry_with_covariance"
    bridge_cmd = [
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
        f'{odom_gz}@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance',
        f'{odom_gz}@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance'
    ]
    bridge_proc = subprocess.Popen(bridge_cmd)
    print("      -> Bridge active.")

    # 3. Mở RViz2
    print("[3/4] Launching RViz2...")
    cfg_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'apf_simulation.rviz')
    if os.path.exists(cfg_file):
        rviz_proc = subprocess.Popen(['rviz2', '-d', cfg_file])
    else:
        rviz_proc = subprocess.Popen(['rviz2'])

    # 4. Spin ROS 2 Node
    print("[4/4] Starting Main ROS 2 Node...")
    print("=" * 70)

    rclpy.init()
    node = FastTrackerStyleNode(obstacles)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            bridge_proc.terminate()
            rviz_proc.terminate()
        except:
            pass

if __name__ == '__main__':
    main()
