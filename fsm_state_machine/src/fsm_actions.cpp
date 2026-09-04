#include "fsm_state_machine/fsm_actions.hpp"

namespace fsm_state_machine
{

FsmActuators::FsmActuators(rclcpp::Node * node)
{
  yaw_rate_pub_ = node->create_publisher<std_msgs::msg::Float32>("cmd/yaw_rate", 10);
  gimbal_state_pub_ = node->create_publisher<std_msgs::msg::UInt8>("gimbal/state_request", 10);
  planner_mode_pub_ = node->create_publisher<std_msgs::msg::UInt8>("planner/mode", 10);
  apf_gain_pub_ = node->create_publisher<std_msgs::msg::Float32>("planner/apf_gain", 10);
  align_error_pub_ = node->create_publisher<std_msgs::msg::Float32>("gimbal/align_error_cmd", 10);
  descent_rate_pub_ = node->create_publisher<std_msgs::msg::Float32>("cmd/vertical_descent_rate", 10);
  disarm_pub_ = node->create_publisher<std_msgs::msg::Bool>("cmd/disarm_request", 10);

  yaw_search_rate_ = node->declare_parameter<float>("yaw_search_rate", 0.3f);
  land_descent_rate_ = node->declare_parameter<float>("land_descent_rate", 0.4f);
}

void FsmActuators::publish_gimbal_state_request(State state)
{
  std_msgs::msg::UInt8 msg;
  msg.data = static_cast<uint8_t>(state);
  gimbal_state_pub_->publish(msg);
}

void FsmActuators::publish_planner_mode(State state)
{
  std_msgs::msg::UInt8 msg;
  msg.data = static_cast<uint8_t>(state);
  planner_mode_pub_->publish(msg);
}

void FsmActuators::action_search()
{
  std_msgs::msg::Float32 yaw_msg;
  yaw_msg.data = yaw_search_rate_;
  yaw_rate_pub_->publish(yaw_msg);
  publish_gimbal_state_request(State::SEARCH);
}

void FsmActuators::action_follow(const SensorInput & /*s*/)
{
  publish_planner_mode(State::FOLLOW);

  std_msgs::msg::Float32 gain_msg;
  gain_msg.data = 1.0f;
  apf_gain_pub_->publish(gain_msg);

  publish_gimbal_state_request(State::FOLLOW);
}

void FsmActuators::action_approach(const SensorInput & s)
{
  publish_planner_mode(State::APPROACH);

  std_msgs::msg::Float32 gain_msg;
  gain_msg.data = 0.5f;
  apf_gain_pub_->publish(gain_msg);

  publish_gimbal_state_request(State::APPROACH);

  std_msgs::msg::Float32 align_msg;
  align_msg.data = s.align_error;
  align_error_pub_->publish(align_msg);
}

void FsmActuators::action_land(const SensorInput & s)
{
  publish_planner_mode(State::LAND);

  std_msgs::msg::Float32 gain_msg;
  gain_msg.data = 0.0f;
  apf_gain_pub_->publish(gain_msg);

  publish_gimbal_state_request(State::LAND);

  std_msgs::msg::Float32 descent_msg;
  descent_msg.data = land_descent_rate_;
  descent_rate_pub_->publish(descent_msg);

  if (s.touchdown) {
    std_msgs::msg::Bool disarm_msg;
    disarm_msg.data = true;
    disarm_pub_->publish(disarm_msg);
  }
}

}  // namespace fsm_state_machine