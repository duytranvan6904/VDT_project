#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "gimbal_control/gimbal_types.hpp"
#include "fsm_state_machine/msg/alt_estimate.hpp"
#include "fsm_state_machine/msg/timeout_flags.hpp"

namespace gimbal_control
{

class GimbalNode : public rclcpp::Node
{
public:
  GimbalNode();

private:
  void on_state_request(const std_msgs::msg::UInt8::SharedPtr msg);
  void on_ekf(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_alt(const fsm_state_machine::msg::AltEstimate::SharedPtr msg);
  void on_timeout_flags(const fsm_state_machine::msg::TimeoutFlags::SharedPtr msg);

  void update();
  GimbalTelemetry build_telemetry() const;
  void publish_angle(float angle_deg);
  void log_debug(float target, float smoothed, float limited) const;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
  rclcpp::Subscription<fsm_state_machine::msg::AltEstimate>::SharedPtr alt_sub_;
  rclcpp::Subscription<fsm_state_machine::msg::TimeoutFlags>::SharedPtr timeout_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry ekf_state_;
  fsm_state_machine::msg::AltEstimate alt_state_;
  fsm_state_machine::msg::TimeoutFlags timeout_flags_;
  GimbalPhase phase_ = GimbalPhase::SEARCH;
  GimbalPhase last_phase_ = GimbalPhase::SEARCH;

  PidState pid_;
  float current_angle_ = 0.0f;
  float max_slew_rate_deg_s_ = 0.0f;
  float land_entry_height_ = 0.0f;
  double last_update_time_ = -1.0;
  bool debug_enabled_ = false;
};

}  // namespace gimbal_control