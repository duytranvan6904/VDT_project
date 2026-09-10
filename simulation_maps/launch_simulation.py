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
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64, Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

# ── Tham số Map ─────────────────────────────────────────────────────────────
NUM_OBS = 15
MAP_SIZE = 20.0
MAX_HEIGHT = 4.0
MODEL_NAME = "x500_depth_0"
HPAD_MODEL_NAME = "hpad_aruco"
HPAD_X = 4.0
HPAD_Y = 0.0
HPAD_Z = 0.02
HPAD_MESH = "/home/duy/VDT_project/PX4-Autopilot/Tools/simulation/gz/models/arucotag/hpad_aruco.dae"

def generate_obstacles():
    half = MAP_SIZE / 2.0
    obs = []
    for _ in range(NUM_OBS):
        cx, cy = random.uniform(-half, half), random.uniform(-half, half)
        # Giữ vùng cất cánh và vùng H-pad không bị vật cản che/mọc đè.
        if math.sqrt(cx**2 + cy**2) < 3.0 or math.hypot(cx - HPAD_X, cy - HPAD_Y) < 2.0:
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
    <include>
      <uri>model://arucotag</uri>
      <name>{HPAD_MODEL_NAME}</name>
      <pose>{HPAD_X:.2f} {HPAD_Y:.2f} {HPAD_Z:.2f} 0 0 0</pose>
    </include>
{models}
  </world>
</sdf>
"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(sdf)

def prepare_world_files():
    """Generate one random world before Gazebo is started."""
    obstacles = generate_obstacles()
    for path in [
        "/home/duy/VDT_project/simulation_maps/gazebo_worlds/obstacle_avoidance.sdf",
        "/home/duy/VDT_project/PX4-Autopilot/Tools/simulation/gz/worlds/obstacle_avoidance.sdf",
    ]:
        write_sdf(obstacles, path)
    print(f"World prepared with {len(obstacles)} obstacles and H-pad at ({HPAD_X}, {HPAD_Y}).")

def load_obstacles_from_world(path):
    """Read the already-loaded SDF so RViz uses exactly Gazebo's map."""
    root = ET.parse(path).getroot()
    result = []
    for model in root.findall('./world/model'):
        model_name = model.attrib.get('name', '')
        if not (model_name.startswith('cyl_') or model_name.startswith('cylinder_obs_')):
            continue
        pose = model.findtext('pose', '').split()
        cylinder = model.find('.//cylinder')
        if len(pose) < 3 or cylinder is None:
            continue
        result.append({
            'cx': float(pose[0]), 'cy': float(pose[1]),
            'r': float(cylinder.findtext('radius', '0.5')),
            'h': float(cylinder.findtext('length', '1.0')),
        })
    return result

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

def discover_gazebo_topics():
    """Return live Gazebo topics and select the x500 camera/odom topics."""
    topics = []
    try:
        result = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True, timeout=5)
        topics = [line.strip() for line in result.stdout.splitlines() if line.strip() and line.startswith('/')]
        interesting = [t for t in topics if any(k in t.lower() for k in ('odom', 'camera', 'depth', 'point'))]
        if interesting:
            print("      -> Gazebo sensor/odom topics detected:")
            for topic in interesting:
                print(f"         {topic}")
        else:
            print("[WARN] No camera/odom topic found from `gz topic -l`; ensure Gazebo is already running.")
    except Exception as exc:
        print(f"[WARN] Could not inspect Gazebo topics before bridging: {exc}")
    return topics

def select_gazebo_topic(topics, exact_names, suffixes, contains=()):
    """Choose the most specific runtime topic, preferring exact names."""
    for name in exact_names:
        if name in topics:
            return name
    candidates = []
    for topic in topics:
        if suffixes and not any(topic.endswith(suffix) for suffix in suffixes):
            continue
        if contains and not all(token in topic for token in contains):
            continue
        candidates.append(topic)
    return sorted(candidates, key=len)[0] if candidates else None

def bridge_argument(gz_topic, ros_type, gz_type, ros_topic, direction='['):
    """Create a ros_gz_bridge argument and an optional ROS topic remap.

    ``[`` is Gazebo -> ROS 2 only.  For camera sensors we use ``@`` because
    the installed ros_gz_bridge versions consistently create the Gazebo
    subscription for the bidirectional form, which is important for lazy
    sensor publishers.  The ROS side remains read-only in this simulation.
    """
    return f'{gz_topic}@{ros_type}{direction}{gz_type}', f'{gz_topic}:={ros_topic}'

def check_ros_gz_backend():
    """Require the bridge binary to be linked against Gazebo Harmonic.

    ROS Humble also provides a Fortress/Ignition build under the same
    ``ros_gz_bridge`` package name.  That binary starts successfully against
    Harmonic but cannot decode ``gz.msgs.*`` traffic, producing ``Unknown
    message type [8]/[9]`` and no Gazebo subscribers.
    """
    try:
        prefix = subprocess.run(
            ['ros2', 'pkg', 'prefix', 'ros_gz_bridge'],
            capture_output=True, text=True, timeout=5, check=True
        ).stdout.strip()
        executable = os.path.join(prefix, 'lib', 'ros_gz_bridge', 'parameter_bridge')
        deps = subprocess.run(
            ['ldd', executable], capture_output=True, text=True, timeout=5, check=True
        ).stdout
    except Exception as exc:
        print(f"[ERROR] Không kiểm tra được ros_gz_bridge backend: {exc}")
        return False

    if 'libgz-msgs' not in deps or 'libgz-transport' not in deps:
        print("[ERROR] ros_gz_bridge hiện là bản Fortress/Ignition, không tương thích Gazebo Harmonic.")
        print("        Cần cài ros-humble-ros-gzharmonic-bridge (thay thế ros-humble-ros-gz-bridge).")
        print("        Kiểm tra lại bằng: ldd " + executable + " | grep -E 'gz-(msgs|transport)|ignition-(msgs|transport)'")
        return False

    print("      -> ros_gz_bridge backend: Gazebo Harmonic (gz-msgs / gz-transport).")
    return True

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
        self.hpad_marker_pub = self.create_publisher(Marker, '/hpad/marker', 10)

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
        self.hpad_pos = [HPAD_X, HPAD_Y, HPAD_Z]
        self.camera_pitch = 0.0
        self.has_odom = False
        self.has_sensor_cloud = False

        # Subscribe Odometry từ Gazebo
        odom_topic = f"/model/{MODEL_NAME}/odometry_with_covariance"
        self.create_subscription(Odometry, odom_topic, self.odom_callback, sensor_qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, sensor_qos)
        self.create_subscription(PoseStamped, '/hpad/ground_truth', self.hpad_callback, 10)
        self.create_subscription(
            Float64,
            f'/model/{MODEL_NAME}/command/gimbal_pitch',
            self.gimbal_pitch_callback,
            10,
        )

        # Subscribe PointCloud2 từ Depth Camera của Drone
        # Keep the canonical topic and common scoped Gazebo fallbacks. The
        # bridge normally exposes /depth_camera/points, but scoped names are
        # useful when Gazebo was started with a model-scoped sensor topic.
        for cloud_topic in [
            '/depth_camera/points',
            f'/model/{MODEL_NAME}/link/camera_link/sensor/StereoOV7251/points',
            f'/model/{MODEL_NAME}/link/camera_link/sensor/StereoOV7251/point_cloud',
        ]:
            self.create_subscription(PointCloud2, cloud_topic, self.sensor_cloud_callback, sensor_qos)

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

    def hpad_callback(self, msg: PoseStamped):
        p = msg.pose.position
        self.hpad_pos = [p.x, p.y, p.z]

    def gimbal_pitch_callback(self, msg: Float64):
        self.camera_pitch = max(
            -math.pi / 2.0,
            min(math.radians(15.0), float(msg.data)),
        )

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

        # Camera frame is rigidly mounted on x500_depth at the Oak-D pose.
        # Publish it under base_link so RViz2 Camera/Image displays can resolve TF.
        camera_tf = TransformStamped()
        camera_tf.header.stamp = now
        camera_tf.header.frame_id = 'base_link'
        camera_tf.child_frame_id = 'camera_link'
        camera_tf.transform.translation.x = 0.12
        camera_tf.transform.translation.y = 0.03
        camera_tf.transform.translation.z = 0.242
        camera_tf.transform.rotation.y = math.sin(self.camera_pitch / 2.0)
        camera_tf.transform.rotation.w = math.cos(self.camera_pitch / 2.0)
        self.tf_broadcaster.sendTransform(camera_tf)

        # OpenCV PnP coordinates use the ROS optical convention:
        # x-right, y-down, z-forward. The sensor itself is slightly offset
        # from camera_link in the Oak-D model.
        optical_tf = TransformStamped()
        optical_tf.header.stamp = now
        optical_tf.header.frame_id = 'camera_link'
        optical_tf.child_frame_id = 'camera_optical_frame'
        optical_tf.transform.translation.x = 0.01233
        optical_tf.transform.translation.y = -0.03
        optical_tf.transform.translation.z = 0.01878
        optical_tf.transform.rotation.x = -0.5
        optical_tf.transform.rotation.y = 0.5
        optical_tf.transform.rotation.z = -0.5
        optical_tf.transform.rotation.w = 0.5
        self.tf_broadcaster.sendTransform(optical_tf)

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

        # Gazebo models are not automatically visible in RViz2. Publish the
        # same ArUco-textured plane as a mesh in the world frame.
        hpad = Marker()
        hpad.header.stamp = now
        hpad.header.frame_id = 'world'
        hpad.ns = 'hpad'
        hpad.id = 0
        hpad.type = Marker.MESH_RESOURCE
        hpad.action = Marker.ADD
        hpad.pose.position.x = self.hpad_pos[0]
        hpad.pose.position.y = self.hpad_pos[1]
        hpad.pose.position.z = self.hpad_pos[2] + 0.003
        hpad.pose.orientation.w = 1.0
        hpad.scale.x = 1.0
        hpad.scale.y = 1.0
        hpad.scale.z = 1.0
        hpad.color.r = 1.0
        hpad.color.g = 1.0
        hpad.color.b = 1.0
        hpad.color.a = 1.0
        hpad.mesh_resource = f'file://{HPAD_MESH}'
        hpad.mesh_use_embedded_materials = True
        self.hpad_marker_pub.publish(hpad)

    def log_status(self):
        odom_str = "CONNECTED" if self.has_odom else "WAITING"
        cloud_str = "RECEIVING" if self.has_sensor_cloud else "WAITING"
        pos_str = f"({self.drone_pos[0]:.2f}, {self.drone_pos[1]:.2f}, {self.drone_pos[2]:.2f})"
        self.get_logger().info(f"[STATUS] Gazebo Odom: [{odom_str}] | Camera PointCloud2: [{cloud_str}] | Drone Pos: {pos_str}")

def main():
    print("=" * 70)
    print("  SIMULATION LAUNCHER (Fast-Tracker & PX4-Avoidance Architecture)")
    print("=" * 70)

    # Generate before PX4/Gazebo starts; never overwrite an active world here.
    if '--generate-world' in sys.argv:
        prepare_world_files()
        return

    print("[1/4] Loading the world already used by Gazebo...")
    world_path = "/home/duy/VDT_project/PX4-Autopilot/Tools/simulation/gz/worlds/obstacle_avoidance.sdf"
    obstacles = load_obstacles_from_world(world_path)
    if not obstacles:
        print("[ERROR] No cylinder obstacles found in the PX4 world.")
        print("        Run: python3 launch_simulation.py --generate-world")
        print("        Then restart PX4/Gazebo before launching this node again.")
        return
    print(f"      -> Loaded {len(obstacles)} obstacles from {world_path}.")

    # 2. Khởi chạy ros_gz_bridge.  Each sensor gets its own process so a
    # malformed/unsupported sensor mapping cannot disable the other sensors.
    print("[2/4] Starting Gazebo Sim -> ROS 2 Bridge...")
    if not check_ros_gz_backend():
        return
    gazebo_topics = discover_gazebo_topics()
    rgb_gz = select_gazebo_topic(gazebo_topics, ['/camera'], ('/camera', '/image'), ('camera',))
    camera_info_gz = select_gazebo_topic(gazebo_topics, ['/camera_info'], ('/camera_info',), ('camera',))
    depth_gz = select_gazebo_topic(gazebo_topics, ['/depth_camera'], ('/depth_camera', '/depth_image'), ('depth',))
    points_gz = select_gazebo_topic(gazebo_topics, ['/depth_camera/points'], ('/depth_camera/points', '/points'), ('point',))
    odom_gz = select_gazebo_topic(gazebo_topics, [f'/model/{MODEL_NAME}/odometry_with_covariance'], ('/odometry_with_covariance',), (MODEL_NAME,))

    if not all((rgb_gz, depth_gz, points_gz, odom_gz)):
        print("[ERROR] Không tìm đủ topic runtime cho bridge:")
        print(f"        RGB={rgb_gz}, CAMERA_INFO={camera_info_gz}, DEPTH={depth_gz}, POINTS={points_gz}, ODOM={odom_gz}")
        print("        Hãy bảo đảm Gazebo đã spawn x500_depth trước khi chạy launcher.")
        return

    odom_bridge_cmd = ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge']
    odom_arg, odom_remap = bridge_argument(odom_gz, 'nav_msgs/msg/Odometry', 'gz.msgs.OdometryWithCovariance', '/odom')
    odom_bridge_cmd += [odom_arg, '--ros-args', '-r', odom_remap]

    sensor_bridge_specs = [
        ('RGB camera', rgb_gz, 'sensor_msgs/msg/Image', 'gz.msgs.Image', '/camera'),
        ('Depth image', depth_gz, 'sensor_msgs/msg/Image', 'gz.msgs.Image', '/depth_camera'),
        ('Depth point cloud', points_gz, 'sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked', '/depth_camera/points'),
    ]
    if camera_info_gz:
        sensor_bridge_specs.insert(
            1, ('RGB camera info', camera_info_gz, 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo', '/camera_info')
        )
    sensor_bridge_procs = []
    for label, source_topic, ros_type, gz_type, ros_topic in sensor_bridge_specs:
        # Use the standard bidirectional form for sensors.  It causes
        # parameter_bridge to subscribe to Gazebo even for lazy publishers.
        bridge_arg, bridge_remap = bridge_argument(
            source_topic, ros_type, gz_type, ros_topic, direction='@'
        )
        bridge_cmd = ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', bridge_arg]
        if source_topic != ros_topic:
            bridge_cmd += ['--ros-args', '-r', bridge_remap]
        print(f"      -> {label} bridge:", ' '.join(bridge_cmd))
        proc = subprocess.Popen(bridge_cmd)
        sensor_bridge_procs.append((label, proc))
        print(f"         started (pid={proc.pid}).")

    print("      -> Odom bridge:", ' '.join(odom_bridge_cmd))
    odom_bridge_proc = subprocess.Popen(odom_bridge_cmd)
    print(f"      -> Odom bridge started (pid={odom_bridge_proc.pid}).")

    # ROS -> Gazebo command for the one-axis camera gimbal. The main node also
    # subscribes to this ROS topic so its TF tree follows the commanded angle.
    gimbal_bridge_arg = (
        f'/model/{MODEL_NAME}/command/gimbal_pitch@'
        'std_msgs/msg/Float64@gz.msgs.Double'
    )
    gimbal_bridge_cmd = [
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', gimbal_bridge_arg
    ]
    print("      -> Gimbal pitch bridge:", ' '.join(gimbal_bridge_cmd))
    gimbal_bridge_proc = subprocess.Popen(gimbal_bridge_cmd)
    print(f"         started (pid={gimbal_bridge_proc.pid}).")

    # Convert Gazebo R_FLOAT32 depth to mono8 for RViz2 Image display.
    depth_node = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'depth_to_image_node.py')
    depth_proc = subprocess.Popen([sys.executable, depth_node])
    time.sleep(1.0)
    for label, proc in sensor_bridge_procs:
        if proc.poll() is not None:
            print(f"[ERROR] {label} ros_gz_bridge exited immediately. Check Gazebo sensor topic names.")
    if odom_bridge_proc.poll() is not None:
        print("[ERROR] Odom ros_gz_bridge exited immediately. Check OdometryWithCovariance support/topic.")
    if gimbal_bridge_proc.poll() is not None:
        print("[ERROR] Gimbal ros_gz_bridge exited immediately. Camera pitch ROS command will not reach Gazebo.")
    if depth_proc.poll() is not None:
        print("[ERROR] depth_to_image_node.py exited immediately.")

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
            for _, sensor_bridge_proc in sensor_bridge_procs:
                sensor_bridge_proc.terminate()
            odom_bridge_proc.terminate()
            gimbal_bridge_proc.terminate()
            depth_proc.terminate()
            rviz_proc.terminate()
        except:
            pass

if __name__ == '__main__':
    main()
