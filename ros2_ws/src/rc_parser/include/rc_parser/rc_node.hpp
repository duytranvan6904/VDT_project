#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "rc_parser/msg/rc_channels_raw.hpp"
#include "rc_parser/msg/rc_fsm_input.hpp"
#include "rc_parser/rc_types.hpp"
#include "rc_parser/rc_uart.hpp"

namespace rc_parser
{

class RcNode : public rclcpp::Node
{
public:
  RcNode();

private:
  void update();
  void publish_fsm_input(const RcChannels & channels);
  void publish_raw(const RcChannels & channels);
  void log_debug(const RcChannels & channels) const;

  std::unique_ptr<SbusUart> uart_;
  RcConfig cfg_;

  rclcpp::Publisher<msg::RcFsmInput>::SharedPtr fsm_input_pub_;
  rclcpp::Publisher<msg::RcChannelsRaw>::SharedPtr raw_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double last_valid_time_;
  bool debug_enabled_;
};

}  // namespace rc_parser