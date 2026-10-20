#include "offboard_manager/offboard_node.hpp"
#include <iostream>

int main(int argc, char ** argv)
{
  std::cout << "[VERBOSE] Starting Offboard Manager Node..." << std::endl;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<offboard_manager::OffboardNode>();
  std::cout << "[VERBOSE] Offboard Manager Node initialized successfully." << std::endl;
  rclcpp::spin(node);
  std::cout << "[VERBOSE] Shutting down Offboard Manager Node..." << std::endl;
  rclcpp::shutdown();
  return 0;
}