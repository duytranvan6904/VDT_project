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

  snapshot_sub_ = create_subscription<input_state_cache::msg::InputSnapshot>(
    "input_cache/snapshot", 10,
    std::bind(&FsmNode::on_snapshot, this, std::placeholders::_1));

  rc_sub_ = create_subscription<rc_parser::msg::RcFsmInput>(
    "rc/fsm_input", 10,
    std::bind(&FsmNode::on_rc, this, std::placeholders::_1));

  rclcpp::QoS killed_qos(1);
  killed_qos.transient_local();
  killed_sub_ = create_subscription<std_msgs::msg::Bool>(
    "system/killed", killed_qos,
    std::bind(&FsmNode::on_killed, this, std::placeholders::_1));

  force_land_sub_ = create_subscription<std_msgs::msg::Bool>(
    "safety/force_land", 10,
    std::bind(&FsmNode::on_force_land, this, std::placeholders::_1));

  state_pub_ = create_publisher<std_msgs::msg::UInt8>("fsm/state", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&FsmNode::update, this));
}

void FsmNode::on_killed(const std_msgs::msg::Bool::SharedPtr msg)
{
  killed_ = msg->data;
}

void FsmNode::on_force_land(const std_msgs::msg::Bool::SharedPtr msg)
{
  force_land_requested_ = msg->data;
}

void FsmNode::on_rc(const rc_parser::msg::RcFsmInput::SharedPtr msg)
{
  rc_input_.land_switch = msg->land_switch;
  rc_input_.kill_switch = msg->kill_switch;
}

void FsmNode::on_snapshot(const input_state_cache::msg::InputSnapshot::SharedPtr msg)
{
  latest_snapshot_ = *msg;
  has_snapshot_ = true;
  last_snapshot_time_ = this->now().seconds();
}

SensorInput FsmNode::build_sensor_input() const
{
  SensorInput s;
  s.marker_detected = latest_snapshot_.marker_detected;
  s.align_error = latest_snapshot_.align_error;
  s.altitude = latest_snapshot_.altitude;
  s.delta_h = latest_snapshot_.delta_h;
  s.d_horiz = latest_snapshot_.d_horiz;
  s.touchdown = latest_snapshot_.touchdown;
  s.valid = latest_snapshot_.valid;
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
  if (killed_) {
    return;
  }

  const double now_sec = this->now().seconds();
  const float dt = last_update_time_ > 0.0 ?
    static_cast<float>(now_sec - last_update_time_) : 0.0f;
  last_update_time_ = now_sec;

  const bool snapshot_timeout = (!has_snapshot_) || ((now_sec - last_snapshot_time_) > 0.2);

  SensorInput s = build_sensor_input();
  if (snapshot_timeout) {
    s.valid = false;
    s.marker_detected = false;
  }

  counters_update_marker_stable(ctx_.counters, s.marker_detected);
  counters_update_marker_lost(ctx_.counters, s.marker_detected, dt);

  const RcInput effective_rc = effective_rc_input(rc_input_, force_land_requested_);

  const bool planner_timeout = snapshot_timeout || latest_snapshot_.planner_timeout;

  std::optional<State> next_state;
  next_state = force_land_transition(ctx_.state, force_land_requested_);
  if (!next_state) {
    next_state = planner_timeout_transition(ctx_.state, planner_timeout);
  }
  if (!next_state) {
    next_state = evaluate_transition(
      ctx_.state, ctx_.counters, effective_rc, s, land_entry_height_);
  }

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
  log_debug(s, effective_rc);
}

}