import rclpy
from takeoff.takeoff_cmd import FlightNode

class BenchNode(FlightNode):
    def gps_ok(self):
        return True

def main():
    rclpy.init()
    node = BenchNode(3.0, 5.0, 20.0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


main()