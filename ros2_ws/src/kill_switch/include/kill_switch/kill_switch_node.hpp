#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include "rc_parser/msg/rc_channels_raw.hpp"
#include "rc_parser/rc_types.hpp"
#include "kill_switch/kill_switch_types.hpp"

namespace kill_switch
{

class KillSwitchNode : public rclcpp::Node
{
public:
  KillSwitchNode();

private:
  void on_rc_channels(const rc_parser::msg::RcChannelsRaw::SharedPtr msg);
  void update();
  void execute();
  void publish_disarm_command();
  void publish_killed_flag(bool killed);
  void log_debug(bool raw_triggered) const;

  rclcpp::Subscription<rc_parser::msg::RcChannelsRaw>::SharedPtr rc_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr killed_flag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rc_parser::RcChannels latest_channels_;
  bool has_channels_;
  rc_parser::RcConfig cfg_;
  KillSwitchContext ctx_;

  int debounce_threshold_;
  bool debug_enabled_;
};

}  // namespace kill_switch