#include "fsm_state_machine/fsm_node.hpp"
#include <iostream>

int main(int argc, char ** argv)
{
  std::cout << "[VERBOSE] Starting FSM State Machine Node..." << std::endl;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fsm_state_machine::FsmNode>();
  std::cout << "[VERBOSE] FSM State Machine Node initialized successfully." << std::endl;
  rclcpp::spin(node);
  std::cout << "[VERBOSE] Shutting down FSM State Machine Node..." << std::endl;
  rclcpp::shutdown();
  return 0;
}