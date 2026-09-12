#include "offboard_manager/offboard_node.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include "offboard_manager/offboard_logic.hpp"

namespace offboard_manager
{

namespace
{
constexpr uint16_t VEHICLE_CMD_DO_SET_MODE = 176;
constexpr uint16_t VEHICLE_CMD_COMPONENT_ARM_DISARM = 400;
constexpr float PX4_CUSTOM_MAIN_MODE_AUTO = 4.0f;
constexpr float PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3.0f;
constexpr float PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0f;
constexpr uint8_t NAV_STATE_OFFBOARD = 14;
constexpr float NAN_F = std::numeric_limits<float>::quiet_NaN();
}  // namespace

OffboardNode::OffboardNode()
: Node("offboard_node")
{
  required_engage_cycles_ = declare_parameter<int>("required_engage_cycles", 10);
  mode_confirm_timeout_cycles_ = declare_parameter<int>("mode_confirm_timeout_cycles", 20);
  health_confirm_timeout_cycles_ = declare_parameter<int>("health_confirm_timeout_cycles", 100);
  data_freshness_timeout_sec_ = declare_parameter<double>("data_freshness_timeout_sec", 1.0);
  max_horizontal_velocity_ = declare_parameter<float>("max_horizontal_velocity", 2.0f);
  max_vertical_velocity_ = declare_parameter<float>("max_vertical_velocity", 1.0f);
  max_yaw_ = declare_parameter<float>("max_yaw", 3.14f);
  if (!std::isfinite(max_horizontal_velocity_) || max_horizontal_velocity_ < 0.0f) {
    max_horizontal_velocity_ = 0.0f;
  }
  if (!std::isfinite(max_vertical_velocity_) || max_vertical_velocity_ < 0.0f) {
    max_vertical_velocity_ = 0.0f;
  }
  if (!std::isfinite(max_yaw_) || max_yaw_ < 0.0f) {
    max_yaw_ = 0.0f;
  }
  watchdog_timeout_sec_ = declare_parameter<double>("watchdog_timeout_sec", 0.5);
  yaw_search_rate_ = declare_parameter<float>("yaw_search_rate", 0.3f);
  land_descent_rate_ = declare_parameter<float>("land_descent_rate", 0.4f);
  debug_enabled_ = declare_parameter<bool>("debug_enabled", false);

  rclcpp::QoS qos(1);
  qos.best_effort();

  rclcpp::QoS killed_qos(1);
  killed_qos.transient_local();

  fsm_state_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "fsm/state", 10, std::bind(&OffboardNode::on_fsm_state, this, std::placeholders::_1));
  planner_sub_ = create_subscription<msg::PlannerOutput>(
    "planner/velocity_setpoint", 10,
    std::bind(&OffboardNode::on_planner_output, this, std::placeholders::_1));
  killed_sub_ = create_subscription<std_msgs::msg::Bool>(
    "system/killed", killed_qos, std::bind(&OffboardNode::on_killed, this, std::placeholders::_1));
  inhibit_sub_ = create_subscription<std_msgs::msg::Bool>(
    "safety/inhibit_offboard", 10, std::bind(&OffboardNode::on_inhibit, this, std::placeholders::_1));
  vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status", qos,
    std::bind(&OffboardNode::on_vehicle_status, this, std::placeholders::_1));
  local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position", qos,
    std::bind(&OffboardNode::on_local_position, this, std::placeholders::_1));
  timeout_sub_ = create_subscription<input_state_cache::msg::TimeoutFlags>(
  "input_cache/timeout_flags", 10,
  std::bind(&OffboardNode::on_timeout_flags, this, std::placeholders::_1));
  control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", qos);
  trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", qos);
  vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", qos);
  status_pub_ = create_publisher<msg::OffboardStatus>("offboard/status", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&OffboardNode::update, this));
}

void OffboardNode::on_fsm_state(const std_msgs::msg::UInt8::SharedPtr msg)
{
  fsm_state_ = static_cast<FsmState>(msg->data);
}

void OffboardNode::on_planner_output(const msg::PlannerOutput::SharedPtr msg)
{
  const bool finite = std::isfinite(msg->vx) && std::isfinite(msg->vy) &&
    std::isfinite(msg->vz) && std::isfinite(msg->yaw);
  if (!finite) {
    planner_output_ = PlannerOutput{};
    return;
  }
  planner_output_.vx = std::clamp(msg->vx, -max_horizontal_velocity_, max_horizontal_velocity_);
  planner_output_.vy = std::clamp(msg->vy, -max_horizontal_velocity_, max_horizontal_velocity_);
  planner_output_.vz = std::clamp(msg->vz, -max_vertical_velocity_, max_vertical_velocity_);
  planner_output_.yaw = std::clamp(msg->yaw, -max_yaw_, max_yaw_);
}

void OffboardNode::on_killed(const std_msgs::msg::Bool::SharedPtr msg)
{
  killed_ = msg->data;
}

void OffboardNode::on_inhibit(const std_msgs::msg::Bool::SharedPtr msg)
{
  inhibited_ = msg->data;
}

void OffboardNode::on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  vehicle_status_ = *msg;
  has_vehicle_status_ = true;
  last_vehicle_status_time_ = this->now().seconds();
}

void OffboardNode::on_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  local_position_ = *msg;
  has_local_position_ = true;
  last_local_position_time_ = this->now().seconds();
}

void OffboardNode::on_timeout_flags(const input_state_cache::msg::TimeoutFlags::SharedPtr msg)
{
  planner_timeout_ = msg->planner_timeout;
}

void OffboardNode::reset_engage_sequence()
{
  ctx_.offboard_active = false;
  ctx_.engage_counter = 0;
  engage_phase_ = EngagePhase::WAIT_SETPOINT_STREAM;
  phase_cycle_count_ = 0;
}

bool OffboardNode::is_vehicle_status_fresh() const
{
  return has_vehicle_status_ &&
         (this->now().seconds() - last_vehicle_status_time_) <= data_freshness_timeout_sec_;
}

bool OffboardNode::is_local_position_fresh() const
{
  return has_local_position_ &&
         (this->now().seconds() - last_local_position_time_) <= data_freshness_timeout_sec_;
}

bool OffboardNode::is_px4_ready() const
{
  const bool ekf_ready = is_local_position_fresh() &&
    local_position_.xy_valid && local_position_.z_valid;
  return is_vehicle_status_fresh() && ekf_ready && !vehicle_status_.failsafe;
}

void OffboardNode::send_heartbeat()
{
  px4_msgs::msg::OffboardControlMode mode_msg;
  mode_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  mode_msg.position = false;
  mode_msg.velocity = true;
  control_mode_pub_->publish(mode_msg);
  ctx_.last_heartbeat_time = this->now().seconds();
}

void OffboardNode::publish_setpoint(const Setpoint & sp)
{
  px4_msgs::msg::TrajectorySetpoint msg;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

  if (sp.type == SetpointType::POSITION) {
    msg.position = {sp.x, sp.y, sp.z};
    msg.velocity = {NAN_F, NAN_F, NAN_F};
  } else {
    msg.position = {NAN_F, NAN_F, NAN_F};
    msg.velocity = {sp.vx, sp.vy, sp.vz};
  }

  if (sp.use_yaw_rate) {
    msg.yaw = NAN_F;
    msg.yawspeed = sp.yaw_rate;
  } else {
    msg.yaw = sp.yaw;
    msg.yawspeed = NAN_F;
  }

  trajectory_pub_->publish(msg);
  ctx_.setpoint = sp;
}

bool OffboardNode::watchdog_check()
{
  const double now_sec = this->now().seconds();
  if (now_sec - ctx_.last_heartbeat_time > watchdog_timeout_sec_) {
    enter_failsafe();
    return false;
  }
  return true;
}

void OffboardNode::enter_failsafe()
{
  reset_engage_sequence();
  publish_vehicle_command(
    VEHICLE_CMD_DO_SET_MODE, 1.0f, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER);
}

void OffboardNode::engage_request()
{
  send_heartbeat();
  publish_setpoint(build_setpoint_search(yaw_search_rate_));
  phase_cycle_count_ += 1;

  switch (engage_phase_) {
    case EngagePhase::WAIT_SETPOINT_STREAM:
      ctx_.engage_counter += 1;
      if (ctx_.engage_counter >= required_engage_cycles_) {
        publish_vehicle_command(VEHICLE_CMD_DO_SET_MODE, 1.0f, PX4_CUSTOM_MAIN_MODE_OFFBOARD);
        engage_phase_ = EngagePhase::WAIT_MODE_CONFIRM;
        phase_cycle_count_ = 0;
      }
      break;

    case EngagePhase::WAIT_MODE_CONFIRM:
      if (is_vehicle_status_fresh() && vehicle_status_.nav_state == NAV_STATE_OFFBOARD) {
        engage_phase_ = EngagePhase::WAIT_HEALTH_CONFIRM;
        phase_cycle_count_ = 0;
      } else if (phase_cycle_count_ >= mode_confirm_timeout_cycles_) {
        reset_engage_sequence();
      }
      break;

    case EngagePhase::WAIT_HEALTH_CONFIRM:
      if (is_px4_ready()) {
        publish_vehicle_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        ctx_.offboard_active = true;
        ctx_.armed = true;
      } else if (phase_cycle_count_ >= health_confirm_timeout_cycles_) {
        reset_engage_sequence();
      }
      break;
  }
}

void OffboardNode::publish_vehicle_command(
  uint16_t command, float param1, float param2, float param3)
{
  px4_msgs::msg::VehicleCommand msg;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  msg.param1 = param1;
  msg.param2 = param2;
  msg.param3 = param3;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  vehicle_command_pub_->publish(msg);
}

void OffboardNode::publish_status()
{
  msg::OffboardStatus status_msg;
  status_msg.offboard_active = ctx_.offboard_active;
  status_msg.heartbeat_age_sec = static_cast<float>(this->now().seconds() - ctx_.last_heartbeat_time);
  status_pub_->publish(status_msg);
}

void OffboardNode::log_debug() const
{
  if (!debug_enabled_) {
    return;
  }
  RCLCPP_INFO(
    get_logger(), "phase=%d active=%d armed=%d nav_state=%d px4_ready=%d",
    static_cast<int>(engage_phase_), ctx_.offboard_active, ctx_.armed,
    vehicle_status_.nav_state, is_px4_ready());
}

void OffboardNode::update()
{
  if (killed_ || inhibited_) {
    reset_engage_sequence();
    publish_status();
    log_debug();
    return;
  }

  if (planner_timeout_) {
    if (ctx_.offboard_active) {
      enter_failsafe();
    } else {
      reset_engage_sequence();
    }
    publish_status();
    log_debug();
    return;
  }

  if (!ctx_.offboard_active) {
    engage_request();
  } else {
    send_heartbeat();
    if (watchdog_check()) {
      const Setpoint sp = planner_timeout_ ?
      build_setpoint_search(yaw_search_rate_) :
      build_setpoint(fsm_state_, planner_output_, yaw_search_rate_, land_descent_rate_);
      publish_setpoint(sp);
    }
  }
  publish_status();
  log_debug();
}

}  // namespace offboard_manager