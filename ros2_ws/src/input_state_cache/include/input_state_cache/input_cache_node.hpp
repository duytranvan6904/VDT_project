#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "input_state_cache/input_cache_types.hpp"
#include "fsm_state_machine/msg/vision_marker.hpp"
#include "fsm_state_machine/msg/alt_estimate.hpp"
#include "input_state_cache/msg/input_snapshot.hpp"
#include "input_state_cache/msg/timeout_flags.hpp"

namespace input_state_cache
{

class InputCacheNode : public rclcpp::Node
{
public:
  InputCacheNode();

private:
  void on_ekf(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_vision(const fsm_state_machine::msg::VisionMarker::SharedPtr msg);
  void on_alt(const fsm_state_machine::msg::AltEstimate::SharedPtr msg);
  void on_planner(const std::shared_ptr<rclcpp::SerializedMessage> msg);

  void update();
  void log_debug(const msg::TimeoutFlags & flags, const msg::InputSnapshot & snapshot) const;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
  rclcpp::Subscription<fsm_state_machine::msg::VisionMarker>::SharedPtr vision_sub_;
  rclcpp::Subscription<fsm_state_machine::msg::AltEstimate>::SharedPtr alt_sub_;
  rclcpp::GenericSubscription::SharedPtr planner_sub_;

  rclcpp::Publisher<msg::InputSnapshot>::SharedPtr snapshot_pub_;
  rclcpp::Publisher<msg::TimeoutFlags>::SharedPtr timeout_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  RawSensors raw_sensors_;
  TopicFreshness freshness_;
  TimeoutThresholds thresholds_;
  bool debug_enabled_;
};

}