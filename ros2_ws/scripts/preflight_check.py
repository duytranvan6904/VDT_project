import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, BatteryStatus

T = {'vehicle_status': (VehicleStatus, ['pre_flight_checks_pass', 'arming_state', 'failsafe', 'nav_state']),
     'vehicle_local_position': (VehicleLocalPosition, ['xy_valid', 'z_valid', 'xy_global', 'dead_reckoning', 'eph', 'epv']),
     'battery_status': (BatteryStatus, ['remaining', 'voltage_v'])}
rclpy.init()
n = Node('preflight')
got = {}
for k, (t, _) in T.items():
    n.create_subscription(t, '/fmu/out/' + k + ('_v1' if k == 'vehicle_status' else ''), lambda m, k=k: got.__setitem__(k, m), Q)
end = time.time() + 4
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
for k, (_, fs) in T.items():
    m = got.get(k)
    for f in fs:
        print(f'{f}: ' + (str(getattr(m, f, '?')) if m else 'KHONG CO DU LIEU'))
rclpy.shutdown()
