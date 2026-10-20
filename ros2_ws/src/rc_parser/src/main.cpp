#include "rc_parser/rc_node.hpp"
#include <iostream>

int main(int argc, char ** argv)
{
  std::cout << "[VERBOSE] Starting RC Parser Node..." << std::endl;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rc_parser::RcNode>();
  std::cout << "[VERBOSE] RC Parser Node initialized successfully." << std::endl;
  rclcpp::spin(node);
  std::cout << "[VERBOSE] Shutting down RC Parser Node..." << std::endl;
  rclcpp::shutdown();
  return 0;
}