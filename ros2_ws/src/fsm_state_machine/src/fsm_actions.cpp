#include "fsm_state_machine/fsm_actions.hpp"

namespace fsm_state_machine
{

FsmActuators::FsmActuators(rclcpp::Node * node)
{
  gimbal_state_pub_ = node->create_publisher<std_msgs::msg::UInt8>("gimbal/state_request", 10);
  planner_mode_pub_ = node->create_publisher<std_msgs::msg::UInt8>("planner/mode", 10);
  apf_gain_pub_ = node->create_publisher<std_msgs::msg::Float32>("planner/apf_gain", 10);
  align_error_pub_ = node->create_publisher<std_msgs::msg::Float32>("gimbal/align_error_cmd", 10);
  descent_rate_pub_ = node->create_publisher<std_msgs::msg::Float32>("cmd/vertical_descent_rate", 10);
  disarm_pub_ = node->create_publisher<std_msgs::msg::Bool>("cmd/disarm_request", 10);

  approach_descent_rate_ = node->declare_parameter<float>("approach_descent_rate", 0.3f);
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

void FsmActuators::publish_descent_rate(float rate)
{
  std_msgs::msg::Float32 msg;
  msg.data = rate;
  descent_rate_pub_->publish(msg);
}

void FsmActuators::action_search()
{
  publish_planner_mode(State::SEARCH);
  publish_descent_rate(0.0f);
  publish_gimbal_state_request(State::SEARCH);
}

void FsmActuators::action_follow(const SensorInput & /*s*/)
{
  publish_planner_mode(State::FOLLOW);

  std_msgs::msg::Float32 gain_msg;
  gain_msg.data = 1.0f;
  apf_gain_pub_->publish(gain_msg);

  publish_gimbal_state_request(State::FOLLOW);
  publish_descent_rate(0.0f);
}

void FsmActuators::action_approach(const SensorInput & s, float land_entry_height)
{
  publish_planner_mode(State::APPROACH);

  std_msgs::msg::Float32 gain_msg;
  gain_msg.data = 0.5f;
  apf_gain_pub_->publish(gain_msg);

  publish_gimbal_state_request(State::APPROACH);

  std_msgs::msg::Float32 align_msg;
  align_msg.data = s.align_error;
  align_error_pub_->publish(align_msg);
  
  const bool can_descend =
    s.marker_detected && s.geometry_valid && s.delta_h >= land_entry_height;
  publish_descent_rate(can_descend ? approach_descent_rate_ : 0.0f);
}

void FsmActuators::action_land(const SensorInput & /*s*/)
{
  publish_planner_mode(State::LAND);

  std_msgs::msg::Float32 gain_msg;
  gain_msg.data = 0.0f;
  apf_gain_pub_->publish(gain_msg);

  publish_gimbal_state_request(State::LAND);

  publish_descent_rate(land_descent_rate_);
}

void FsmActuators::action_complete()
{
  std_msgs::msg::Bool disarm_msg;
  disarm_msg.data = true;
  disarm_pub_->publish(disarm_msg);
}

}  // namespace fsm_state_machine