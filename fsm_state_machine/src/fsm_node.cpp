#include "fsm_state_machine/fsm_node.hpp"
#include <cmath>
#include "fsm_state_machine/fsm_logic.hpp"

namespace fsm_state_machine
{

FsmNode::FsmNode()
: Node("fsm_node"), actuators_(this), last_update_time_(-1.0)
{
  land_entry_height_ = declare_parameter<float>("land_entry_height", 0.5f);
  debug_enabled_ = declare_parameter<bool>("debug_enabled", false);

  ekf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "hpad/state_filtered", 10, std::bind(&FsmNode::on_ekf, this, std::placeholders::_1));
  vision_sub_ = create_subscription<msg::VisionMarker>(
    "hpad/pose", 10, std::bind(&FsmNode::on_vision, this, std::placeholders::_1));
  alt_sub_ = create_subscription<msg::AltEstimate>(
    "alt_estimator/state", 10, std::bind(&FsmNode::on_alt, this, std::placeholders::_1));
  rc_sub_ = create_subscription<msg::RcFsmInput>(
    "rc/fsm_input", 10, std::bind(&FsmNode::on_rc, this, std::placeholders::_1));
  timeout_sub_ = create_subscription<msg::TimeoutFlags>(
    "input_cache/timeout_flags", 10,
    std::bind(&FsmNode::on_timeout_flags, this, std::placeholders::_1));

  state_pub_ = create_publisher<std_msgs::msg::UInt8>("fsm/state", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&FsmNode::update, this));
}

void FsmNode::on_ekf(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  ekf_state_ = *msg;
}

void FsmNode::on_vision(const msg::VisionMarker::SharedPtr msg)
{
  vision_state_ = *msg;
}

void FsmNode::on_alt(const msg::AltEstimate::SharedPtr msg)
{
  alt_state_ = *msg;
}

void FsmNode::on_rc(const msg::RcFsmInput::SharedPtr msg)
{
  rc_input_.land_switch = msg->land_switch;
  rc_input_.kill_switch = msg->kill_switch;
}

void FsmNode::on_timeout_flags(const msg::TimeoutFlags::SharedPtr msg)
{
  timeout_flags_.ekf_timeout = msg->ekf_timeout;
  timeout_flags_.vision_timeout = msg->vision_timeout;
}

SensorInput FsmNode::build_sensor_input() const
{
  SensorInput s;
  s.marker_detected = vision_state_.marker_visible;
  s.align_error = vision_state_.pixel_align_error;
  s.altitude = alt_state_.altitude;
  s.delta_h = static_cast<float>(ekf_state_.pose.pose.position.z) - alt_state_.altitude;
  s.d_horiz = std::hypot(
    ekf_state_.pose.pose.position.x, ekf_state_.pose.pose.position.y);
  s.touchdown = alt_state_.touchdown_flag;
  s.valid = true;
  return s;
}

void FsmNode::publish_state()
{
  std_msgs::msg::UInt8 msg;
  msg.data = static_cast<uint8_t>(ctx_.state);
  state_pub_->publish(msg);
}

void FsmNode::log_debug(const SensorInput & s, const RcInput & rc) const
{
  if (!debug_enabled_) {
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "state=%d stable=%d lost_t=%.2f align_err=%.3f alt=%.2f delta_h=%.2f land_sw=%d",
    static_cast<int>(ctx_.state), ctx_.counters.marker_stable_count,
    ctx_.counters.marker_lost_time, s.align_error, s.altitude, s.delta_h, rc.land_switch);
}

void FsmNode::update()
{
  const double now_sec = this->now().seconds();
  const float dt = last_update_time_ > 0.0 ?
    static_cast<float>(now_sec - last_update_time_) : 0.0f;
  last_update_time_ = now_sec;

  const SensorInput s = sensor_validate(build_sensor_input(), timeout_flags_);

  counters_update_marker_stable(ctx_.counters, s.marker_detected);
  counters_update_marker_lost(ctx_.counters, s.marker_detected, dt);

  const auto next_state = evaluate_transition(
    ctx_.state, ctx_.counters, rc_input_, s, land_entry_height_);
  if (next_state && *next_state != ctx_.state) {
    ctx_.state = *next_state;
    counters_reset(ctx_.counters);
    ctx_.last_transition_time = now_sec;
  }

  switch (ctx_.state) {
    case State::SEARCH:
      actuators_.action_search();
      break;
    case State::FOLLOW:
      actuators_.action_follow(s);
      break;
    case State::APPROACH:
      actuators_.action_approach(s);
      break;
    case State::LAND:
      actuators_.action_land(s);
      break;
    case State::COMPLETE:
      break;
  }

  publish_state();
  log_debug(s, rc_input_);
}

}  // namespace fsm_state_machine