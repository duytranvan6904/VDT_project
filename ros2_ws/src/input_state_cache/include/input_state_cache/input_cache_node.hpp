#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "input_state_cache/input_cache_types.hpp"
#include "input_state_cache/msg/timeout_flags.hpp"

namespace input_state_cache
{

class InputCacheNode : public rclcpp::Node
{
public:
  InputCacheNode();

private:
  void on_ekf(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_vision(const std::shared_ptr<rclcpp::SerializedMessage> msg);
  void on_alt(const std::shared_ptr<rclcpp::SerializedMessage> msg);
  void on_planner(const std::shared_ptr<rclcpp::SerializedMessage> msg);

  void update();
  void log_debug(const msg::TimeoutFlags & flags) const;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
  rclcpp::GenericSubscription::SharedPtr vision_sub_;
  rclcpp::GenericSubscription::SharedPtr alt_sub_;
  rclcpp::GenericSubscription::SharedPtr planner_sub_;
  rclcpp::Publisher<msg::TimeoutFlags>::SharedPtr timeout_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  TopicFreshness freshness_;
  TimeoutThresholds thresholds_;
  bool debug_enabled_;
};

}  // namespace input_state_cache