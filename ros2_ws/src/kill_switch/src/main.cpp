#include "kill_switch/kill_switch_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kill_switch::KillSwitchNode>());
  rclcpp::shutdown();
  return 0;
}