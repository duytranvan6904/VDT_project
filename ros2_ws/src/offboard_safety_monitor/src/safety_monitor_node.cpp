#include "offboard_safety_monitor/safety_monitor_node.hpp"
#include "offboard_safety_monitor/safety_logic.hpp"
#include <cmath>

namespace offboard_safety_monitor
{

namespace
{
constexpr uint16_t VEHICLE_CMD_DO_SET_MODE = 176;
constexpr uint8_t NAV_STATE_OFFBOARD = 14;
constexpr uint8_t PX4_CUSTOM_MAIN_MODE_AUTO = 4;
constexpr uint8_t PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3;
constexpr uint8_t PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5;
}  // namespace

SafetyMonitorNode::SafetyMonitorNode()
: Node("safety_monitor_node")
{
  thresholds_.battery_warning_frac = declare_parameter<float>("battery_warning_frac", 0.3f);
  thresholds_.battery_critical_frac = declare_parameter<float>("battery_critical_frac", 0.15f);
  thresholds_.offboard_hold_timeout = declare_parameter<double>("offboard_hold_timeout", 1.0);
  thresholds_.offboard_rtl_timeout = declare_parameter<double>("offboard_rtl_timeout", 5.0);
  debug_enabled_ = declare_parameter<bool>("debug_enabled", false);
  force_land_latched_ = declare_parameter<bool>("force_land_latched", true);
  
  data_freshness_timeout_sec_ = declare_parameter<double>("data_freshness_timeout_sec", 1.0);
  status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status", 10,
    std::bind(&SafetyMonitorNode::on_vehicle_status, this, std::placeholders::_1));
  local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position", 10,
    std::bind(&SafetyMonitorNode::on_local_position, this, std::placeholders::_1));
  battery_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>(
    "/fmu/out/battery_status", 10,
    std::bind(&SafetyMonitorNode::on_battery, this, std::placeholders::_1));
  offboard_status_sub_ = create_subscription<offboard_manager::msg::OffboardStatus>(
    "offboard/status", 10,
    std::bind(&SafetyMonitorNode::on_offboard_status, this, std::placeholders::_1));

  vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", 10);
  inhibit_pub_ = create_publisher<std_msgs::msg::Bool>("safety/inhibit_offboard", 10);
  force_land_pub_ = create_publisher<std_msgs::msg::Bool>("safety/force_land", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(200), std::bind(&SafetyMonitorNode::update, this));
}

void SafetyMonitorNode::on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  vehicle_status_ = *msg;
}

void SafetyMonitorNode::on_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  local_position_ = *msg;
  has_local_position_ = true;
  last_local_position_time_ = this->now().seconds();
}

void SafetyMonitorNode::on_battery(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
  battery_status_ = *msg;
  has_battery_ = true;
  last_battery_time_ = this->now().seconds();
}

void SafetyMonitorNode::on_offboard_status(
  const offboard_manager::msg::OffboardStatus::SharedPtr msg)
{
  offboard_status_ = *msg;
}

void SafetyMonitorNode::update()
{
  const double now_sec = this->now().seconds();

  const bool ekf_fresh = is_fresh(
    has_local_position_, last_local_position_time_, now_sec, data_freshness_timeout_sec_);
  const bool battery_fresh = is_fresh(
    has_battery_, last_battery_time_, now_sec, data_freshness_timeout_sec_);

  const bool rc_override = check_rc_override(
    offboard_status_.offboard_active, vehicle_status_.nav_state, NAV_STATE_OFFBOARD);
  const bool ekf_healthy = check_ekf_health(
    local_position_.xy_valid, local_position_.z_valid, ekf_fresh);
  const FailsafeLevel battery_level = check_battery_failsafe(
    battery_status_.remaining, battery_fresh, thresholds_);
  const FailsafeLevel escalate_level = offboard_watchdog_escalate(
    offboard_status_.heartbeat_age_sec, thresholds_);

  const FailsafeLevel level = evaluate_failsafe_level(
    rc_override, ekf_healthy, battery_level, escalate_level);

  ctx_.active_failsafe = level;
  if (level == FailsafeLevel::BATTERY_WARNING) {
    ctx_.force_land_requested = true;
  } else if (!force_land_latched_ && level == FailsafeLevel::NONE && battery_fresh &&
    std::isfinite(battery_status_.remaining) &&
    battery_status_.remaining >= thresholds_.battery_warning_frac)
  {
    ctx_.force_land_requested = false;
  }

  execute_action(level);
  publish_force_land(ctx_.force_land_requested);
  log_debug(level);
}

void SafetyMonitorNode::execute_action(FailsafeLevel level)
{
  switch (level) {
    case FailsafeLevel::RC_OVERRIDE:
      publish_inhibit(true);
      break;
    case FailsafeLevel::EKF_UNHEALTHY:
      publish_mode_command(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER);
      publish_inhibit(true);
      break;
    case FailsafeLevel::BATTERY_WARNING:
      break;
    case FailsafeLevel::BATTERY_CRITICAL:
      publish_mode_command(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTL);
      publish_inhibit(true);
      break;
    case FailsafeLevel::OFFBOARD_LOST_SHORT:
      publish_mode_command(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER);
      publish_inhibit(true);
      break;
    case FailsafeLevel::OFFBOARD_LOST_LONG:
      publish_mode_command(PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTL);
      publish_inhibit(true);
      break;
    case FailsafeLevel::NONE:
      publish_inhibit(false);
      break;
  }
}

void SafetyMonitorNode::publish_mode_command(uint8_t main_mode, uint8_t sub_mode)
{
  px4_msgs::msg::VehicleCommand msg;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  msg.param1 = 1.0f;
  msg.param2 = static_cast<float>(main_mode);
  msg.param3 = static_cast<float>(sub_mode);
  msg.command = VEHICLE_CMD_DO_SET_MODE;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  vehicle_command_pub_->publish(msg);
}

void SafetyMonitorNode::publish_inhibit(bool inhibit)
{
  std_msgs::msg::Bool msg;
  msg.data = inhibit;
  inhibit_pub_->publish(msg);
}

void SafetyMonitorNode::publish_force_land(bool force_land)
{
  std_msgs::msg::Bool msg;
  msg.data = force_land;
  force_land_pub_->publish(msg);
}

void SafetyMonitorNode::log_debug(FailsafeLevel level) const
{
  if (!debug_enabled_) {
    return;
  }
  RCLCPP_INFO(
    get_logger(), "level=%d force_land=%d battery=%.2f heartbeat_age=%.2f",
    static_cast<int>(level), ctx_.force_land_requested,
    battery_status_.remaining, offboard_status_.heartbeat_age_sec);
}

}  // namespace offboard_safety_monitor