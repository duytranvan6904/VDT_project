#include "rc_parser/rc_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rc_parser::RcNode>());
  rclcpp::shutdown();
  return 0;
}