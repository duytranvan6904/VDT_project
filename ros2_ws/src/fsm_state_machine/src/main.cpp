#include "fsm_state_machine/fsm_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fsm_state_machine::FsmNode>());
  rclcpp::shutdown();
  return 0;
}