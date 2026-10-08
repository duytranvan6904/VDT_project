#pragma once
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "input_state_cache/input_cache_types.hpp"
#include "vdt_msgs/msg/vision_marker.hpp"
#include "vdt_msgs/msg/alt_estimate.hpp"
#include "vdt_msgs/msg/input_snapshot.hpp"
#include "vdt_msgs/msg/timeout_flags.hpp"

namespace input_state_cache
{

namespace msg = vdt_msgs::msg;  
class InputCacheNode : public rclcpp::Node
{
public:
  InputCacheNode();

private:
  void on_ekf(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_tracking(const std_msgs::msg::String::SharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_vision(const vdt_msgs::msg::VisionMarker::SharedPtr msg);
  void on_alt(const vdt_msgs::msg::AltEstimate::SharedPtr msg);
  void on_planner(const std::shared_ptr<rclcpp::SerializedMessage> msg);
  void on_landing_touchdown(const std_msgs::msg::Bool::SharedPtr msg);

  bool frame_ok(const std::string & frame_id, const char * source);
  void update();
  void log_debug(const msg::TimeoutFlags & flags, const msg::InputSnapshot & snapshot) const;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tracking_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<vdt_msgs::msg::VisionMarker>::SharedPtr vision_sub_;
  rclcpp::Subscription<vdt_msgs::msg::AltEstimate>::SharedPtr alt_sub_;
  rclcpp::GenericSubscription::SharedPtr planner_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr landing_touchdown_sub_;

  rclcpp::Publisher<msg::InputSnapshot>::SharedPtr snapshot_pub_;
  rclcpp::Publisher<msg::TimeoutFlags>::SharedPtr timeout_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  RawSensors raw_sensors_;
  TopicFreshness freshness_;
  TimeoutThresholds thresholds_;
  std::string world_frame_;
  bool debug_enabled_;
  bool use_landing_touchdown_{false};
  bool landing_touchdown_{false};
  double last_landing_touchdown_time_{0.0};
  double landing_touchdown_timeout_{0.5};
};

}  // namespace input_state_cache