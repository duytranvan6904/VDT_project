#include "offboard_manager/offboard_node.hpp"
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
constexpr float NAN_F = std::numeric_limits<float>::quiet_NaN();
}  // namespace

OffboardNode::OffboardNode()
: Node("offboard_node")
{
  required_engage_cycles_ = declare_parameter<int>("required_engage_cycles", 10);
  watchdog_timeout_sec_ = declare_parameter<double>("watchdog_timeout_sec", 0.5);
  yaw_search_rate_ = declare_parameter<float>("yaw_search_rate", 0.3f);
  land_descent_rate_ = declare_parameter<float>("land_descent_rate", 0.4f);
  debug_enabled_ = declare_parameter<bool>("debug_enabled", false);

  rclcpp::QoS qos(1);
  qos.best_effort();

  fsm_state_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "fsm/state", 10, std::bind(&OffboardNode::on_fsm_state, this, std::placeholders::_1));
  planner_sub_ = create_subscription<msg::PlannerOutput>(
    "planner/velocity_setpoint", 10,
    std::bind(&OffboardNode::on_planner_output, this, std::placeholders::_1));

  control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", qos);
  trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", qos);
  vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", qos);
  
  inhibit_sub_ = create_subscription<std_msgs::msg::Bool>(
  "safety/inhibit_offboard", 10,
  std::bind(&OffboardNode::on_inhibit, this, std::placeholders::_1));
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
  planner_output_.vx = msg->vx;
  planner_output_.vy = msg->vy;
  planner_output_.vz = msg->vz;
  planner_output_.yaw = msg->yaw;
}

void OffboardNode::on_inhibit(const std_msgs::msg::Bool::SharedPtr msg)
{
  inhibited_ = msg->data;
}

void OffboardNode::publish_status()
{
  msg::OffboardStatus msg;
  msg.offboard_active = ctx_.offboard_active;
  msg.heartbeat_age_sec = static_cast<float>(this->now().seconds() - ctx_.last_heartbeat_time);
  status_pub_->publish(msg);
}

void OffboardNode::send_heartbeat()
{
  px4_msgs::msg::OffboardControlMode mode_msg;
  mode_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  mode_msg.position = true;
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
  ctx_.offboard_active = false;
  publish_vehicle_command(
    VEHICLE_CMD_DO_SET_MODE, 1.0f, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER);
}

void OffboardNode::engage_request()
{
  if (ctx_.offboard_active) {
    return;
  }
  send_heartbeat();
  publish_setpoint(build_setpoint_search(yaw_search_rate_));
  ctx_.engage_counter += 1;

  if (ctx_.engage_counter >= required_engage_cycles_) {
    publish_vehicle_command(VEHICLE_CMD_DO_SET_MODE, 1.0f, PX4_CUSTOM_MAIN_MODE_OFFBOARD);
    publish_vehicle_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    ctx_.offboard_active = true;
    ctx_.armed = true;
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

void OffboardNode::log_debug() const
{
  if (!debug_enabled_) {
    return;
  }
  RCLCPP_INFO(
    get_logger(), "active=%d armed=%d engage_cnt=%d vx=%.2f vy=%.2f vz=%.2f",
    ctx_.offboard_active, ctx_.armed, ctx_.engage_counter,
    ctx_.setpoint.vx, ctx_.setpoint.vy, ctx_.setpoint.vz);
}

void OffboardNode::update()
{
  if (inhibited_) {
    ctx_.offboard_active = false;
    ctx_.engage_counter = 0;
    publish_status();
    log_debug();
    return;
  }

  if (!ctx_.offboard_active) {
    engage_request();
  } else {
    send_heartbeat();
    if (watchdog_check()) {
      const Setpoint sp = build_setpoint(
        fsm_state_, planner_output_, yaw_search_rate_, land_descent_rate_);
      publish_setpoint(sp);
    }
  }
  publish_status();
  log_debug();
}

}  // namespace offboard_manager