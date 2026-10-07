#include "fsm_state_machine/fsm_node.hpp"
#include <cmath>
#include "fsm_state_machine/fsm_logic.hpp"

namespace fsm_state_machine
{

FsmNode::FsmNode()
: Node("fsm_node"), actuators_(this), last_update_time_(-1.0)
{
  params_.land_entry_height = declare_parameter<float>("land_entry_height", 0.5f);
  params_.enter_follow_cycles = declare_parameter<int>("enter_follow_cycles", 10);
  params_.follow_lost_timeout = declare_parameter<float>("follow_lost_timeout", 2.5f);
  params_.approach_lost_timeout = declare_parameter<float>("approach_lost_timeout", 1.5f);
  params_.align_threshold = declare_parameter<float>("align_threshold", 0.3f);
  params_.land_entry_cycles = declare_parameter<int>("land_entry_cycles", 5);
  params_.yaw_settle_rate = declare_parameter<float>("yaw_settle_rate", 0.1f);
  params_.land_requires_rearm = declare_parameter<bool>("land_requires_rearm", true);
  params_.ignore_planner_timeout = declare_parameter<bool>("ignore_planner_timeout", false);
  debug_enabled_ = declare_parameter<bool>("debug_enabled", false);

  snapshot_sub_ = create_subscription<vdt_msgs::msg::InputSnapshot>(
    "input_cache/snapshot", 10,
    std::bind(&FsmNode::on_snapshot, this, std::placeholders::_1));

  rc_sub_ = create_subscription<vdt_msgs::msg::RcFsmInput>(
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

void FsmNode::on_rc(const vdt_msgs::msg::RcFsmInput::SharedPtr msg)
{
  rc_input_.land_switch = msg->land_switch;
  rc_input_.kill_switch = msg->kill_switch;
}

void FsmNode::on_snapshot(const vdt_msgs::msg::InputSnapshot::SharedPtr msg)
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
  s.yaw_rate = latest_snapshot_.yaw_rate;
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
    "state=%d stable=%d lost_t=%.2f land_ok=%d land_inh=%d align_err=%.3f alt=%.2f delta_h=%.2f "
    "yaw_rate=%.3f geom=%d land_sw=%d",
    static_cast<int>(ctx_.state), ctx_.counters.marker_stable_count,
    ctx_.counters.marker_lost_time, ctx_.counters.land_ok_count,
    static_cast<int>(ctx_.counters.land_inhibit), s.align_error, s.altitude, s.delta_h,
    s.yaw_rate, static_cast<int>(s.geometry_valid), rc.land_switch);
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
  s = sensor_validate(s, TimeoutFlags{});

  counters_update_marker_stable(ctx_.counters, search_entry_ok(s, params_));
  counters_update_marker_lost(ctx_.counters, s.marker_detected, dt);
  counters_update_land_ok(ctx_.counters, ctx_.state, land_entry_ok(s, params_));

  const RcInput effective_rc = effective_rc_input(rc_input_, force_land_requested_);

  const bool planner_timeout = snapshot_timeout || latest_snapshot_.planner_timeout;
  if (params_.ignore_planner_timeout) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "ignore_planner_timeout is enabled");
  }

  std::optional<State> next_state;
  next_state = force_land_transition(ctx_.state, force_land_requested_);
  if (!next_state) {
    next_state = planner_timeout_transition(ctx_.state, planner_timeout, params_);
  }
  if (!next_state) {
    next_state = evaluate_transition(
      ctx_.state, ctx_.counters, effective_rc, s, params_);
  }

  const State prev_state = ctx_.state;
  if (next_state && *next_state != ctx_.state) {
    ctx_.state = *next_state;
    counters_reset(ctx_.counters);
    ctx_.last_transition_time = now_sec;
  }
  counters_update_land_inhibit(ctx_.counters, prev_state, ctx_.state, effective_rc);

  switch (ctx_.state) {
    case State::SEARCH:
      actuators_.action_search();
      break;
    case State::FOLLOW:
      actuators_.action_follow(s);
      break;
    case State::APPROACH:
      actuators_.action_approach(s, params_.land_entry_height);
      break;
    case State::LAND:
      actuators_.action_land(s);
      break;
    case State::COMPLETE:
      actuators_.action_complete();
      break;
  }

  publish_state();
  log_debug(s, effective_rc);
}

}