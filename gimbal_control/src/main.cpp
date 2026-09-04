#include "gimbal_control/gimbal_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gimbal_control::GimbalNode>());
  rclcpp::shutdown();
  return 0;
}