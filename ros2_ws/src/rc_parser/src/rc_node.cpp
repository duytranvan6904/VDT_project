#include "rc_parser/rc_node.hpp"
#include "rc_parser/rc_logic.hpp"

namespace rc_parser
{

RcNode::RcNode()
: Node("rc_node"), last_valid_time_(0.0)
{
  const std::string device = declare_parameter<std::string>("serial_device", "/dev/ttyUSB0");
  const int baudrate = declare_parameter<int>("baudrate", 100000);

  cfg_.land_channel = declare_parameter<int>("land_channel", 4);
  cfg_.kill_channel = declare_parameter<int>("kill_channel", 5);
  cfg_.low_threshold = declare_parameter<int>("low_threshold", 1200);
  cfg_.high_threshold = declare_parameter<int>("high_threshold", 1800);
  cfg_.frame_timeout = declare_parameter<double>("frame_timeout", 0.5);
  debug_enabled_ = declare_parameter<bool>("debug_enabled", false);

  uart_ = std::make_unique<SbusUart>(device, baudrate);
  if (!uart_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Khong mo duoc serial device %s", device.c_str());
  }

  fsm_input_pub_ = create_publisher<msg::RcFsmInput>("rc/fsm_input", 10);
  raw_pub_ = create_publisher<msg::RcChannelsRaw>("rc/channels_raw", 10);

  timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&RcNode::update, this));
}

void RcNode::update()
{
  std::array<uint8_t, SBUS_FRAME_LEN> frame{};
  const double now_sec = this->now().seconds();
  RcChannels channels;
  if (uart_->is_open() && uart_->read_frame(frame)) {
    channels = sbus_decode_frame(frame);
  } else {
    channels.failsafe = true;
  }

  if (channels.valid && !channels.failsafe) {
    last_valid_time_ = now_sec;
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

}  // namespace rc_parser