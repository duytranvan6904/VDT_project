#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "vdt_msgs/msg/rc_channels_raw.hpp"
#include "vdt_msgs/msg/rc_fsm_input.hpp"
#include "rc_parser/rc_types.hpp"
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>

namespace rc_parser
{
namespace msg = vdt_msgs::msg;
class RcNode : public rclcpp::Node
{
public:
  RcNode();

private:
  void update();
  void on_manual_control(const px4_msgs::msg::ManualControlSetpoint::SharedPtr in);
  void on_failsafe_flags(const px4_msgs::msg::FailsafeFlags::SharedPtr in);
  void publish_fsm_input(const RcChannels & channels);
  void publish_raw(const RcChannels & channels);
  void log_debug(const RcChannels & channels) const;

  RcConfig cfg_;

  rclcpp::Publisher<msg::RcFsmInput>::SharedPtr fsm_input_pub_;
  rclcpp::Publisher<msg::RcChannelsRaw>::SharedPtr raw_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_sub_;
  rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr flags_sub_;
  RcChannels latest_channels_;
  bool has_msg_ = false;
  bool signal_lost_ = false;

  double last_valid_time_;
  bool debug_enabled_;
};

}  // namespace rc_parser