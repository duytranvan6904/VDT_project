#include "input_state_cache/input_cache_node.hpp"
#include "input_state_cache/input_cache_logic.hpp"

namespace input_state_cache
{

InputCacheNode::InputCacheNode()
: Node("input_cache_node")
{
  thresholds_.ekf_timeout_sec = declare_parameter<double>("ekf_timeout_sec", 0.5);
  thresholds_.vision_timeout_sec = declare_parameter<double>("vision_timeout_sec", 0.5);
  thresholds_.alt_timeout_sec = declare_parameter<double>("alt_timeout_sec", 0.5);
  thresholds_.planner_timeout_sec = declare_parameter<double>("planner_timeout_sec", 1.0);
  debug_enabled_ = declare_parameter<bool>("debug_enabled", false);

  ekf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "hpad/state_filtered", 10, std::bind(&InputCacheNode::on_ekf, this, std::placeholders::_1));
  vision_sub_ = create_subscription<fsm_state_machine::msg::VisionMarker>(
    "hpad/pose", 10, std::bind(&InputCacheNode::on_vision, this, std::placeholders::_1));
  alt_sub_ = create_subscription<fsm_state_machine::msg::AltEstimate>(
    "alt_estimator/state", 10, std::bind(&InputCacheNode::on_alt, this, std::placeholders::_1));
  planner_sub_ = create_subscription<offboard_manager::msg::PlannerOutput>(
    "planner/velocity_setpoint", 10,
    std::bind(&InputCacheNode::on_planner, this, std::placeholders::_1));

  timeout_pub_ = create_publisher<fsm_state_machine::msg::TimeoutFlags>(
    "input_cache/timeout_flags", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&InputCacheNode::update, this));
}

void InputCacheNode::on_ekf(const nav_msgs::msg::Odometry::SharedPtr)
{
  freshness_.last_ekf_time = this->now().seconds();
}

void InputCacheNode::on_vision(const fsm_state_machine::msg::VisionMarker::SharedPtr)
{
  freshness_.last_vision_time = this->now().seconds();
}

void InputCacheNode::on_alt(const fsm_state_machine::msg::AltEstimate::SharedPtr)
{
  freshness_.last_alt_time = this->now().seconds();
}

void InputCacheNode::on_planner(const offboard_manager::msg::PlannerOutput::SharedPtr)
{
  freshness_.last_planner_time = this->now().seconds();
}

void InputCacheNode::update()
{
  const double now_sec = this->now().seconds();
  const auto flags = compute_timeout_flags(freshness_, thresholds_, now_sec);
  timeout_pub_->publish(flags);
  log_debug(flags);
}

void InputCacheNode::log_debug(const fsm_state_machine::msg::TimeoutFlags & flags) const
{
  if (!debug_enabled_) {
    return;
  }
  RCLCPP_INFO(
    get_logger(), "ekf=%d vision=%d alt=%d planner=%d",
    flags.ekf_timeout, flags.vision_timeout, flags.alt_timeout, flags.planner_timeout);
}

}  // namespace input_state_cache