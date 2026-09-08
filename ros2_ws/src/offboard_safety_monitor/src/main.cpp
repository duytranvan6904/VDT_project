#include "offboard_safety_monitor/safety_monitor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<offboard_safety_monitor::SafetyMonitorNode>());
  rclcpp::shutdown();
  return 0;
}