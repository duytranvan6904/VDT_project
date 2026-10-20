#include "offboard_safety_monitor/safety_monitor_node.hpp"
#include <iostream>

int main(int argc, char ** argv)
{
  std::cout << "[VERBOSE] Starting Offboard Safety Monitor Node..." << std::endl;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<offboard_safety_monitor::SafetyMonitorNode>();
  std::cout << "[VERBOSE] Offboard Safety Monitor Node initialized successfully." << std::endl;
  rclcpp::spin(node);
  std::cout << "[VERBOSE] Shutting down Offboard Safety Monitor Node..." << std::endl;
  rclcpp::shutdown();
  return 0;
}