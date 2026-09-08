#include "gimbal_control/gimbal_node.hpp"
#include <cmath>
#include "gimbal_control/gimbal_logic.hpp"

namespace gimbal_control
{

GimbalNode::GimbalNode()
: Node("gimbal_node")
{
  pid_.kp = declare_parameter<float>("kp", 1.0f);
  pid_.ki = declare_parameter<float>("ki", 0.0f);
  pid_.kd = declare_parameter<float>("kd", 0.1f);
  pid_.out_min = declare_parameter<float>("out_min_deg", -90.0f);
  pid_.out_max = declare_parameter<float>("out_max_deg", 90.0f);

  max_slew_rate_deg_s_ = declare_parameter<float>("max_slew_rate_deg_s", 60.0f);
  land_entry_height_ = declare_parameter<float>("land_entry_height", 0.5f);
  debug_enabled_ = declare_parameter<bool>("debug_enabled", false);

  state_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "gimbal/state_request", 10,
    std::bind(&GimbalNode::on_state_request, this, std::placeholders::_1));
  ekf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "hpad/state_filtered", 10, std::bind(&GimbalNode::on_ekf, this, std::placeholders::_1));
  alt_sub_ = create_subscription<fsm_state_machine::msg::AltEstimate>(
    "alt_estimator/state", 10, std::bind(&GimbalNode::on_alt, this, std::placeholders::_1));
  timeout_sub_ = create_subscription<fsm_state_machine::msg::TimeoutFlags>(
    "input_cache/timeout_flags", 10,
    std::bind(&GimbalNode::on_timeout_flags, this, std::placeholders::_1));

  angle_pub_ = create_publisher<std_msgs::msg::Float32>("gimbal/target_angle_deg", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&GimbalNode::update, this));
}

void GimbalNode::on_state_request(const std_msgs::msg::UInt8::SharedPtr msg)
{
  phase_ = static_cast<GimbalPhase>(msg->data);
}

void GimbalNode::on_ekf(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  ekf_state_ = *msg;
}

void GimbalNode::on_alt(const fsm_state_machine::msg::AltEstimate::SharedPtr msg)
{
  alt_state_ = *msg;
}

void GimbalNode::on_timeout_flags(const fsm_state_machine::msg::TimeoutFlags::SharedPtr msg)
{
  timeout_flags_ = *msg;
}

GimbalTelemetry GimbalNode::build_telemetry() const
{
  GimbalTelemetry t;
  t.delta_h = static_cast<float>(ekf_state_.pose.pose.position.z) - alt_state_.altitude;
  t.d_horiz = std::hypot(
    ekf_state_.pose.pose.position.x, ekf_state_.pose.pose.position.y);
  t.altitude = alt_state_.altitude;
  t.valid = true;
  return t;
}

void GimbalNode::publish_angle(float angle_deg)
{
  std_msgs::msg::Float32 msg;
  msg.data = angle_deg;
  angle_pub_->publish(msg);
}

void GimbalNode::log_debug(float target, float smoothed, float limited) const
{
  if (!debug_enabled_) {
    return;
  }
  RCLCPP_INFO(
    get_logger(), "phase=%d target=%.2f smoothed=%.2f limited=%.2f",
    static_cast<int>(phase_), target, smoothed, limited);
}

void GimbalNode::update()
{
  const double now_sec = this->now().seconds();
  const float dt = last_update_time_ > 0.0 ?
    static_cast<float>(now_sec - last_update_time_) : 0.0f;
  last_update_time_ = now_sec;

  const GimbalTelemetry t = telemetry_validate(build_telemetry(), timeout_flags_);
  if (!t.valid) {
    return;
  }

  if (phase_ != last_phase_) {
    pid_reset(pid_);
    last_phase_ = phase_;
  }

  const float target = gimbal_target_angle(phase_, t, land_entry_height_);
  const float smoothed = pid_update(pid_, target, current_angle_, dt);
  const float limited = clamp_slew_rate(current_angle_, smoothed, max_slew_rate_deg_s_, dt);

  publish_angle(limited);
  current_angle_ = limited;
  log_debug(target, smoothed, limited);
}

}  // namespace gimbal_control