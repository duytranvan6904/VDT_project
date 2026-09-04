#!/usr/bin/env python3
"""
1-to-1 Map Synchronizer for Gazebo Sim (3D SDF) and RViz2 (PointCloud2).
Generates identical 3D cylinder obstacles in both environments simultaneously.
"""

import math
import random
import struct
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class MapSynchronizerNode(Node):
    def __init__(self, obstacles):
        super().__init__('map_synchronizer')
        self.publisher_ = self.create_publisher(PointCloud2, '/map_generator/global_cloud', 10)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tf()

        self.obstacles = obstacles
        self.points = self.build_pointcloud_from_obstacles(obstacles)

        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info(f"1-to-1 Map Synchronizer active! Publishing {len(self.points)} points to RViz2.")

    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'map'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_static_broadcaster.sendTransform(t)

    def build_pointcloud_from_obstacles(self, obstacles, resolution=0.15):
        points = []
        for obs in obstacles:
            cx, cy, radius, height = obs['cx'], obs['cy'], obs['radius'], obs['height']
            z = 0.0
            while z <= height:
                num_steps = max(int(2 * math.pi * radius / resolution), 8)
                for i in range(num_steps):
                    theta = i * (2 * math.pi / num_steps)
                    px = cx + radius * math.cos(theta)
                    py = cy + radius * math.sin(theta)
                    points.append((px, py, z))
                z += resolution
        return points

    def publish_map(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'world'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_data = []
        for p in self.points:
            cloud_data.append(struct.pack('fff', p[0], p[1], p[2]))

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(self.points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(self.points)
        msg.is_dense = True
        msg.data = b"".join(cloud_data)

        self.publisher_.publish(msg)

def generate_random_obstacle_data(num_obs=15, map_size=20.0, max_height=4.0):
    obstacles = []
    half_size = map_size / 2.0
    for _ in range(num_obs):
        cx = random.uniform(-half_size, half_size)
        cy = random.uniform(-half_size, half_size)
        radius = random.uniform(0.35, 0.65)
        height = random.uniform(2.5, max_height)

        if math.sqrt(cx**2 + cy**2) < 3.0:
            continue

        obstacles.append({
            'cx': cx,
            'cy': cy,
            'radius': radius,
            'height': height,
            'color': (random.uniform(0.2, 0.9), random.uniform(0.2, 0.9), random.uniform(0.2, 0.9))
        })
    return obstacles

def write_gazebo_sdf(obstacles, output_file):
    obstacles_sdf = ""
    for idx, obs in enumerate(obstacles, 1):
        cx, cy, radius, height, (r, g, b) = obs['cx'], obs['cy'], obs['radius'], obs['height'], obs['color']
        obstacles_sdf += f"""
    <!-- Obstacle Pillar {idx} -->
    <model name="cylinder_obs_{idx}">
      <static>true</static>
      <pose>{cx:.2f} {cy:.2f} {height/2.0:.2f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{radius:.2f}</radius>
              <length>{height:.2f}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{radius:.2f}</radius>
              <length>{height:.2f}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
            <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

    sdf_content = f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="obstacle_avoidance">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>

    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>

    <scene>
      <grid>true</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
    </scene>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics" />
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands" />
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster" />
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu" />
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure" />
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer" />
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat" />

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>200 200</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>200 200</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
{obstacles_sdf}
  </world>
</sdf>
"""
    with open(output_file, 'w') as f:
        f.write(sdf_content)

def main():
    obstacles = generate_random_obstacle_data(num_obs=15, map_size=20.0, max_height=4.0)

    # 1. Update Gazebo SDF Files
    gz_local = "/home/duy/VDT_project/simulation_maps/gazebo_worlds/obstacle_avoidance.sdf"
    gz_px4 = "/home/duy/VDT_project/PX4-Autopilot/Tools/simulation/gz/worlds/obstacle_avoidance.sdf"
    write_gazebo_sdf(obstacles, gz_local)
    if os.path.exists(os.path.dirname(gz_px4)):
        write_gazebo_sdf(obstacles, gz_px4)
    print("[+] Gazebo 3D World updated with synchronized obstacles.")

    # 2. Start ROS 2 PointCloud Publisher for RViz2
    rclpy.init()
    node = MapSynchronizerNode(obstacles)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
