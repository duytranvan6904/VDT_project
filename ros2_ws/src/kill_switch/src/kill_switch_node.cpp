#include "kill_switch/kill_switch_node.hpp"
#include "kill_switch/kill_switch_logic.hpp"
#include "rc_parser/rc_logic.hpp"

namespace kill_switch
{

namespace
{
constexpr uint16_t VEHICLE_CMD_COMPONENT_ARM_DISARM = 400;
constexpr float FORCE_MAGIC_NUMBER = 21196.0f;
}  // namespace

KillSwitchNode::KillSwitchNode()
: Node("kill_switch_node"), has_channels_(false)
{
  cfg_.kill_channel = declare_parameter<int>("kill_channel", 5);
  cfg_.low_threshold = declare_parameter<int>("low_threshold", 1200);
  cfg_.high_threshold = declare_parameter<int>("high_threshold", 1800);
  debounce_threshold_ = declare_parameter<int>("debounce_threshold", 3);
  debug_enabled_ = declare_parameter<bool>("debug_enabled", false);

  rc_sub_ = create_subscription<rc_parser::msg::RcChannelsRaw>(
    "rc/channels_raw", 10, std::bind(&KillSwitchNode::on_rc_channels, this, std::placeholders::_1));

  vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", 10);

  rclcpp::QoS killed_qos(1);
  killed_qos.transient_local();
  killed_flag_pub_ = create_publisher<std_msgs::msg::Bool>("system/killed", killed_qos);
  publish_killed_flag(false);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&KillSwitchNode::update, this));
}

void KillSwitchNode::on_rc_channels(const rc_parser::msg::RcChannelsRaw::SharedPtr msg)
{
  for (size_t i = 0; i < latest_channels_.ch.size(); ++i) {
    latest_channels_.ch[i] = msg->ch[i];
  }
  latest_channels_.valid = msg->valid;
  latest_channels_.failsafe = msg->failsafe;
  has_channels_ = true;
}

void KillSwitchNode::update()
{
  if (!has_channels_ || ctx_.triggered) {
    return;
  }

  const bool raw_triggered = latest_channels_.valid &&
    rc_parser::rc_get_kill_switch(latest_channels_, cfg_);

  if (kill_switch_debounce(ctx_, raw_triggered, debounce_threshold_)) {
    execute();
  }
  log_debug(raw_triggered);
}

void KillSwitchNode::execute()
{
  ctx_.triggered = true;
  ctx_.last_trigger_time = this->now().seconds();
  publish_disarm_command();
  publish_killed_flag(true);
}

void KillSwitchNode::publish_disarm_command()
{
  px4_msgs::msg::VehicleCommand msg;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  msg.param1 = 0.0f;
  msg.param2 = FORCE_MAGIC_NUMBER;
  msg.command = VEHICLE_CMD_COMPONENT_ARM_DISARM;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  vehicle_command_pub_->publish(msg);
}

void KillSwitchNode::publish_killed_flag(bool killed)
{
  std_msgs::msg::Bool msg;
  msg.data = killed;
  killed_flag_pub_->publish(msg);
}

void KillSwitchNode::log_debug(bool raw_triggered) const
{
  if (!debug_enabled_) {
    return;
  }
  RCLCPP_INFO(
    get_logger(), "raw=%d debounce=%d triggered=%d",
    raw_triggered, ctx_.debounce_count, ctx_.triggered);
}

}  // namespace kill_switch