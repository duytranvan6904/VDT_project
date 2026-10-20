#include "kill_switch/kill_switch_node.hpp"
#include <iostream>

int main(int argc, char ** argv)
{
  std::cout << "[VERBOSE] Starting Kill Switch Node..." << std::endl;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<kill_switch::KillSwitchNode>();
  std::cout << "[VERBOSE] Kill Switch Node initialized successfully." << std::endl;
  rclcpp::spin(node);
  std::cout << "[VERBOSE] Shutting down Kill Switch Node..." << std::endl;
  rclcpp::shutdown();
  return 0;
}