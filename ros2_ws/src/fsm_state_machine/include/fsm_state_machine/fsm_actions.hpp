#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "fsm_state_machine/fsm_types.hpp"

namespace fsm_state_machine
{

class FsmActuators
{
public:
  explicit FsmActuators(rclcpp::Node * node);

  void action_search();
  void action_follow(const SensorInput & s);
  void action_approach(const SensorInput & s, float land_entry_height);
  void action_land(const SensorInput & s);
  void action_complete();

private:
  void publish_gimbal_state_request(State state);
  void publish_planner_mode(State state);
  void publish_descent_rate(float rate);

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gimbal_state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr planner_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr apf_gain_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr align_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr descent_rate_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr disarm_pub_;

  float land_descent_rate_;
  float approach_descent_rate_;
};

}  // namespace fsm_state_machine