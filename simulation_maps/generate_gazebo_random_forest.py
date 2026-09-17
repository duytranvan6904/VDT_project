#!/usr/bin/env python3
"""
Generator to build a 3D Random Forest SDF World for Gazebo Sim (gz_x500)
Optimized for 60 FPS performance and full PX4 EKF2 Sensor/GPS compatibility.
"""

import math
import random
import os

def generate_sdf_world(num_obs=15, map_size=20.0, max_height=4.0, output_file="obstacle_avoidance.sdf"):
    half_size = map_size / 2.0

    obstacles_sdf = ""
    count = 0

    # ── 1. Đặt các chướng ngại vật kiểm thử CỐ ĐỊNH ngay trên hành lang Drone -> H-Pad ──
    # Drone ở (0, 0), H-Pad ở (4.0, -2.0). Đặt 1 trụ ngay giữa đường (4.0, -0.75) để test APF né:
    test_obstacles = [
        {"cx": 4.0, "cy": -0.75, "radius": 0.40, "height": 3.8, "r": 0.9, "g": 0.2, "b": 0.2},
    ]

    for obs in test_obstacles:
        count += 1
        obstacles_sdf += f"""
    <!-- Corridor Test Obstacle {count} -->
    <model name="cylinder_obs_{count}">
      <static>true</static>
      <pose>{obs['cx']:.2f} {obs['cy']:.2f} {obs['height']/2.0:.2f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{obs['radius']:.2f}</radius>
              <length>{obs['height']:.2f}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{obs['radius']:.2f}</radius>
              <length>{obs['height']:.2f}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{obs['r']:.2f} {obs['g']:.2f} {obs['b']:.2f} 1</ambient>
            <diffuse>{obs['r']:.2f} {obs['g']:.2f} {obs['b']:.2f} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

    # ── 2. Sinh ngẫu nhiên các trụ chướng ngại vật khác xung quanh ──
    for i in range(num_obs):
        cx = random.uniform(-half_size, half_size)
        cy = random.uniform(-half_size, half_size)
        radius = random.uniform(0.3, 0.5)
        height = random.uniform(2.5, max_height)

        # Chỉ chừa bán kính nhỏ 1.2m quanh vị trí drone cất cánh (0, 0)
        # và 0.8m quanh H-Pad (5.0, 0.0)
        if math.hypot(cx, cy) < 4.0 or math.hypot(cx - 5.0, cy) < 2.0:
            continue
        # Tránh đè lên 2 trụ test cố định
        if math.hypot(cx - 2.2, cy - 0.35) < 1.0 or math.hypot(cx - 3.6, cy + 0.30) < 1.0:
            continue

        r_color = random.uniform(0.2, 0.9)
        g_color = random.uniform(0.2, 0.9)
        b_color = random.uniform(0.2, 0.9)

        count += 1
        obstacles_sdf += f"""
    <!-- Obstacle Pillar {count} -->
    <model name="cylinder_obs_{count}">
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
            <ambient>{r_color:.2f} {g_color:.2f} {b_color:.2f} 1</ambient>
            <diffuse>{r_color:.2f} {g_color:.2f} {b_color:.2f} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

    sdf_content = f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="obstacle_avoidance">
    <!-- Physics System for PX4 -->
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>

    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>

    <!-- Optimized Scene: Shadows Disabled for Fast GPU Performance -->
    <scene>
      <grid>true</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
    </scene>

    <!-- Explicit Gazebo Sim GUI: keeps orbit/pan/zoom controls available. -->
    <gui fullscreen="false">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.7 0.7 0.7</background_color>
        <camera_pose>10 -14 10 0 0.55 0.55</camera_pose>
        <camera_clip><near>0.1</near><far>250</far></camera_clip>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager" />
      <plugin filename="InteractiveViewControl" name="Interactive view control" />
      <plugin filename="CameraTracking" name="Camera Tracking" />
      <plugin filename="EntityContextMenuPlugin" name="Entity context menu" />
    </gui>

    <!-- Essential Gazebo Sim System Plugins -->
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

    <!-- Sun Directional Light -->
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane -->
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

    <!-- Spherical Coordinates for PX4 EKF2 GPS Location -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>

    <!-- H-pad: the existing PX4 arucotag model is an ArUco image plane. -->
    <include>
      <uri>model://arucotag</uri>
      <name>hpad_aruco</name>
      <pose>4.00 -2.00 0.02 0 0 0</pose>
    </include>
{obstacles_sdf}
  </world>
</sdf>
"""

    with open(output_file, 'w') as f:
        f.write(sdf_content)

    print(f"[+] Generated lightweight, EKF2-compatible Gazebo world with {count} pillars: {output_file}")

if __name__ == '__main__':
    target = "/home/duy/VDT_project/simulation_maps/gazebo_worlds/obstacle_avoidance.sdf"
    generate_sdf_world(num_obs=15, map_size=20.0, max_height=4.0, output_file=target)

    # Also update PX4-Autopilot gz worlds directory
    px4_gz_target = "/home/duy/VDT_project/PX4-Autopilot/Tools/simulation/gz/worlds/obstacle_avoidance.sdf"
    if os.path.exists(os.path.dirname(px4_gz_target)):
        generate_sdf_world(num_obs=15, map_size=20.0, max_height=4.0, output_file=px4_gz_target)
