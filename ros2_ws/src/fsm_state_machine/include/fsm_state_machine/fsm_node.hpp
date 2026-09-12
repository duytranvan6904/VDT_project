#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "fsm_state_machine/fsm_types.hpp"
#include "fsm_state_machine/fsm_actions.hpp"
#include "input_state_cache/msg/input_snapshot.hpp"
#include "rc_parser/msg/rc_fsm_input.hpp"

namespace fsm_state_machine
{

class FsmNode : public rclcpp::Node
{
public:
  FsmNode();

private:
  void on_snapshot(const input_state_cache::msg::InputSnapshot::SharedPtr msg);
  void on_rc(const rc_parser::msg::RcFsmInput::SharedPtr msg);
  void on_killed(const std_msgs::msg::Bool::SharedPtr msg);
  void on_force_land(const std_msgs::msg::Bool::SharedPtr msg);

  void update();
  SensorInput build_sensor_input() const;
  void publish_state();
  void log_debug(const SensorInput & s, const RcInput & rc) const;

  rclcpp::Subscription<input_state_cache::msg::InputSnapshot>::SharedPtr snapshot_sub_;
  rclcpp::Subscription<rc_parser::msg::RcFsmInput>::SharedPtr rc_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr killed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr force_land_sub_;

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  FsmActuators actuators_;
  FsmContext ctx_;
  RcInput rc_input_;
  input_state_cache::msg::InputSnapshot latest_snapshot_;

  bool has_snapshot_ = false;
  double last_snapshot_time_ = -1.0;
  bool killed_ = false;
  bool force_land_requested_ = false;
  float land_entry_height_ = 0.5f;
  double last_update_time_ = -1.0;
  bool debug_enabled_ = false;
};

}