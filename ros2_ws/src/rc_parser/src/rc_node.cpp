#include "rc_parser/rc_node.hpp"
#include "rc_parser/rc_logic.hpp"
#include <algorithm>
#include <cmath>

namespace rc_parser
{

namespace
{
int to_pwm(float v)
{
  if (!std::isfinite(v)) {
    return 1500;
  }
  return static_cast<int>(1500.0f + 500.0f * std::clamp(v, -1.0f, 1.0f));
}
}  // namespace

RcNode::RcNode()
: Node("rc_node"), last_valid_time_(0.0)
{

  cfg_.land_channel = declare_parameter<int>("land_channel", 4);
  cfg_.kill_channel = declare_parameter<int>("kill_channel", 5);
  cfg_.low_threshold = declare_parameter<int>("low_threshold", 1200);
  cfg_.high_threshold = declare_parameter<int>("high_threshold", 1800);
  cfg_.frame_timeout = declare_parameter<double>("frame_timeout", 0.5);
  debug_enabled_ = declare_parameter<bool>("debug_enabled", false);

  manual_sub_ = create_subscription<px4_msgs::msg::ManualControlSetpoint>(
  "/fmu/out/manual_control_setpoint", rclcpp::SensorDataQoS(),
  std::bind(&RcNode::on_manual_control, this, std::placeholders::_1));
  flags_sub_ = create_subscription<px4_msgs::msg::FailsafeFlags>(
  "/fmu/out/failsafe_flags", rclcpp::SensorDataQoS(),
  std::bind(&RcNode::on_failsafe_flags, this, std::placeholders::_1));

  fsm_input_pub_ = create_publisher<msg::RcFsmInput>("rc/fsm_input", 10);
  raw_pub_ = create_publisher<msg::RcChannelsRaw>("rc/channels_raw", 10);

  timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&RcNode::update, this));
}

void RcNode::update()
{
  const double now_sec = this->now().seconds();
  RcChannels channels = latest_channels_;
  if (!has_msg_) {
    channels.failsafe = true;
  }
  channels = rc_validate(channels, cfg_, last_valid_time_, now_sec);

  publish_fsm_input(channels);
  publish_raw(channels);
  log_debug(channels);
}

void RcNode::publish_fsm_input(const RcChannels & channels)
{
  msg::RcFsmInput msg;
  msg.land_switch = channels.valid && rc_get_land_trigger(channels, cfg_);
  msg.kill_switch = channels.valid && rc_get_kill_switch(channels, cfg_);
  fsm_input_pub_->publish(msg);
}

void RcNode::publish_raw(const RcChannels & channels)
{
  msg::RcChannelsRaw msg;
  for (size_t i = 0; i < channels.ch.size(); ++i) {
    msg.ch[i] = static_cast<int16_t>(channels.ch[i]);
  }
  msg.valid = channels.valid;
  msg.failsafe = channels.failsafe;
  raw_pub_->publish(msg);
}

void RcNode::log_debug(const RcChannels & channels) const
{
  if (!debug_enabled_) {
    return;
  }
  RCLCPP_INFO(
    get_logger(), "valid=%d failsafe=%d land=%d kill=%d",
    channels.valid, channels.failsafe,
    rc_get_land_trigger(channels, cfg_), rc_get_kill_switch(channels, cfg_));
}

void RcNode::on_manual_control(const px4_msgs::msg::ManualControlSetpoint::SharedPtr in)
{
  RcChannels ch;
  ch.ch[0] = to_pwm(in->roll);
  ch.ch[1] = to_pwm(in->pitch);
  ch.ch[2] = to_pwm(in->throttle);
  ch.ch[3] = to_pwm(in->yaw);
  const float aux[6] = {in->aux1, in->aux2, in->aux3, in->aux4, in->aux5, in->aux6};
  for (size_t i = 0; i < 6; ++i) {
    ch.ch[4 + i] = to_pwm(aux[i]);
  }
  for (size_t i = 10; i < ch.ch.size(); ++i) {
    ch.ch[i] = 1500;
  }
  const bool source_ok = in->valid &&
    in->data_source == px4_msgs::msg::ManualControlSetpoint::SOURCE_RC;
  ch.failsafe = !source_ok || signal_lost_;
  ch.valid = !ch.failsafe;
  latest_channels_ = ch;
  has_msg_ = true;
  if (ch.valid) {
    last_valid_time_ = this->now().seconds();
  }
}

void RcNode::on_failsafe_flags(const px4_msgs::msg::FailsafeFlags::SharedPtr in)
{
  signal_lost_ = in->manual_control_signal_lost;
}

}  // namespace rc_parser