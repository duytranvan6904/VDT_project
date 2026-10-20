#include "gimbal_control/gimbal_node.hpp"
#include <iostream>

int main(int argc, char ** argv)
{
  std::cout << "[VERBOSE] Starting Gimbal Control Node..." << std::endl;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gimbal_control::GimbalNode>();
  std::cout << "[VERBOSE] Gimbal Control Node initialized successfully." << std::endl;
  rclcpp::spin(node);
  std::cout << "[VERBOSE] Shutting down Gimbal Control Node..." << std::endl;
  rclcpp::shutdown();
  return 0;
}