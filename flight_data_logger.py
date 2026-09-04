import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition, TrajectorySetpoint, VehicleStatus, VehicleLandDetected


class FlightDataLogger(Node):
    def __init__(self):
        super().__init__('flight_data_logger')

        self.declare_parameter('output_dir', '/home/user/flight_logs')
        output_dir = self.get_parameter('output_dir').value
        os.makedirs(output_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(output_dir, f'flight_log_{timestamp}.csv')

        self.local_position = None
        self.setpoint = None
        self.vehicle_status = None
        self.land_detected = None

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_position_cb, qos_profile
        )
        self.create_subscription(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
            self.setpoint_cb, qos_profile
        )
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.status_cb, qos_profile
        )
        self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self.land_detected_cb, qos_profile
        )

        self.csv_file = open(self.log_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp',
            'pos_x', 'pos_y', 'pos_z',
            'vel_x', 'vel_y', 'vel_z',
            'sp_x', 'sp_y', 'sp_z',
            'sp_vx', 'sp_vy', 'sp_vz',
            'nav_state', 'arming_state',
            'landed'
        ])

        self.create_timer(0.05, self.log_row)

        self.get_logger().info(f'Logging to {self.log_path}')

    def local_position_cb(self, msg):
        self.local_position = msg

    def setpoint_cb(self, msg):
        self.setpoint = msg

    def status_cb(self, msg):
        self.vehicle_status = msg

    def land_detected_cb(self, msg):
        self.land_detected = msg

    def log_row(self):
        if self.local_position is None:
            return

        pos = self.local_position
        sp = self.setpoint
        status = self.vehicle_status
        land = self.land_detected

        sp_pos = sp.position if sp is not None else [float('nan')] * 3
        sp_vel = sp.velocity if sp is not None else [float('nan')] * 3

        self.csv_writer.writerow([
            pos.timestamp,
            pos.x, pos.y, pos.z,
            pos.vx, pos.vy, pos.vz,
            sp_pos[0], sp_pos[1], sp_pos[2],
            sp_vel[0], sp_vel[1], sp_vel[2],
            status.nav_state if status is not None else '',
            status.arming_state if status is not None else '',
            land.landed if land is not None else ''
        ])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = FlightDataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()