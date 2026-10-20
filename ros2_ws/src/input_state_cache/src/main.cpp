#include "input_state_cache/input_cache_node.hpp"
#include <iostream>

int main(int argc, char ** argv)
{
  std::cout << "[VERBOSE] Starting Input State Cache Node..." << std::endl;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<input_state_cache::InputCacheNode>();
  std::cout << "[VERBOSE] Input State Cache Node initialized successfully." << std::endl;
  rclcpp::spin(node);
  std::cout << "[VERBOSE] Shutting down Input State Cache Node..." << std::endl;
  rclcpp::shutdown();
  return 0;
}