#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "fsm_state_machine/fsm_actions.hpp"
#include "fsm_state_machine/fsm_types.hpp"
#include "fsm_state_machine/msg/alt_estimate.hpp"
#include "fsm_state_machine/msg/rc_fsm_input.hpp"
#include "fsm_state_machine/msg/timeout_flags.hpp"
#include "fsm_state_machine/msg/vision_marker.hpp"

namespace fsm_state_machine
{

class FsmNode : public rclcpp::Node
{
public:
  FsmNode();

private:
  void on_ekf(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_vision(const msg::VisionMarker::SharedPtr msg);
  void on_alt(const msg::AltEstimate::SharedPtr msg);
  void on_rc(const msg::RcFsmInput::SharedPtr msg);
  void on_timeout_flags(const msg::TimeoutFlags::SharedPtr msg);

  void update();
  SensorInput build_sensor_input() const;
  void publish_state();
  void log_debug(const SensorInput & s, const RcInput & rc) const;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
  rclcpp::Subscription<msg::VisionMarker>::SharedPtr vision_sub_;
  rclcpp::Subscription<msg::AltEstimate>::SharedPtr alt_sub_;
  rclcpp::Subscription<msg::RcFsmInput>::SharedPtr rc_sub_;
  rclcpp::Subscription<msg::TimeoutFlags>::SharedPtr timeout_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  FsmActuators actuators_;
  FsmContext ctx_;

  nav_msgs::msg::Odometry ekf_state_;
  msg::VisionMarker vision_state_;
  msg::AltEstimate alt_state_;
  RcInput rc_input_;
  TimeoutFlags timeout_flags_;

  float land_entry_height_;
  double last_update_time_;
  bool debug_enabled_;
};

}  // namespace fsm_state_machine