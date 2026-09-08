#include "input_state_cache/input_cache_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<input_state_cache::InputCacheNode>());
  rclcpp::shutdown();
  return 0;
}