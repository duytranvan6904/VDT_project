#include "input_state_cache/input_cache_node.hpp"
#include "input_state_cache/input_cache_logic.hpp"

namespace input_state_cache
{

InputCacheNode::InputCacheNode()
: Node("input_cache_node")
{
  thresholds_.ekf_timeout_sec = declare_parameter<double>("ekf_timeout_sec", 0.5);
  thresholds_.odom_timeout_sec = declare_parameter<double>("odom_timeout_sec", 0.5);
  thresholds_.vision_timeout_sec = declare_parameter<double>("vision_timeout_sec", 0.5);
  thresholds_.alt_timeout_sec = declare_parameter<double>("alt_timeout_sec", 0.5);
  thresholds_.planner_timeout_sec = declare_parameter<double>("planner_timeout_sec", 1.0);
  world_frame_ = declare_parameter<std::string>("world_frame", "world");
  debug_enabled_ = declare_parameter<bool>("debug_enabled", false);

  rclcpp::QoS sensor_qos(5);
  sensor_qos.best_effort();

  ekf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "hpad/state_filtered", 10, std::bind(&InputCacheNode::on_ekf, this, std::placeholders::_1));
  tracking_sub_ = create_subscription<std_msgs::msg::String>(
    "ekf/tracking_mode", 10, std::bind(&InputCacheNode::on_tracking, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", sensor_qos, std::bind(&InputCacheNode::on_odom, this, std::placeholders::_1));
  vision_sub_ = create_subscription<vdt_msgs::msg::VisionMarker>(
    "vision/marker", 10, std::bind(&InputCacheNode::on_vision, this, std::placeholders::_1));
  alt_sub_ = create_subscription<vdt_msgs::msg::AltEstimate>(
    "alt_estimator/state", 10, std::bind(&InputCacheNode::on_alt, this, std::placeholders::_1));
  planner_sub_ = create_generic_subscription(
    "planner/velocity_setpoint", "offboard_manager/msg/PlannerOutput", rclcpp::QoS(10),
    std::bind(&InputCacheNode::on_planner, this, std::placeholders::_1));

  snapshot_pub_ = create_publisher<msg::InputSnapshot>("input_cache/snapshot", 10);
  timeout_pub_ = create_publisher<msg::TimeoutFlags>("input_cache/timeout_flags", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&InputCacheNode::update, this));
}

bool InputCacheNode::frame_ok(const std::string & frame_id, const char * source)
{
  if (world_frame_.empty() || frame_id == world_frame_) {
    return true;
  }
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 2000, "%s frame_id '%s' != '%s', dropped",
    source, frame_id.c_str(), world_frame_.c_str());
  return false;
}

void InputCacheNode::on_ekf(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!frame_ok(msg->header.frame_id, "hpad/state_filtered")) {
    return;
  }
  raw_sensors_.ekf.x = static_cast<float>(msg->pose.pose.position.x);
  raw_sensors_.ekf.y = static_cast<float>(msg->pose.pose.position.y);
  raw_sensors_.ekf.z = static_cast<float>(msg->pose.pose.position.z);
  raw_sensors_.ekf.vx = static_cast<float>(msg->twist.twist.linear.x);
  raw_sensors_.ekf.vy = static_cast<float>(msg->twist.twist.linear.y);
  raw_sensors_.ekf.vz = static_cast<float>(msg->twist.twist.linear.z);
  raw_sensors_.ekf.received = true;
  freshness_.last_ekf_time = this->now().seconds();
}

void InputCacheNode::on_tracking(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & m = msg->data;
  raw_sensors_.tracking.expired =
    !(m == "TRACKING" || m == "PREDICTING" || m == "PREDICTING_DEGRADED");
  raw_sensors_.tracking.received = true;
  freshness_.last_tracking_time = this->now().seconds();
}

void InputCacheNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!frame_ok(msg->header.frame_id, "odom")) {
    return;
  }
  raw_sensors_.odom.x = static_cast<float>(msg->pose.pose.position.x);
  raw_sensors_.odom.y = static_cast<float>(msg->pose.pose.position.y);
  raw_sensors_.odom.z = static_cast<float>(msg->pose.pose.position.z);
  raw_sensors_.odom.received = true;
  freshness_.last_odom_time = this->now().seconds();
}

void InputCacheNode::on_vision(const vdt_msgs::msg::VisionMarker::SharedPtr msg)
{
  raw_sensors_.vision.marker_visible = msg->marker_visible;
  raw_sensors_.vision.pixel_align_error = msg->pixel_align_error;
  raw_sensors_.vision.received = true;
  freshness_.last_vision_time = this->now().seconds();
}

void InputCacheNode::on_alt(const vdt_msgs::msg::AltEstimate::SharedPtr msg)
{
  raw_sensors_.alt.altitude = msg->altitude;
  raw_sensors_.alt.touchdown_flag = msg->touchdown_flag;
  raw_sensors_.alt.received = true;
  freshness_.last_alt_time = this->now().seconds();
}

void InputCacheNode::on_planner(const std::shared_ptr<rclcpp::SerializedMessage>)
{
  freshness_.last_planner_time = this->now().seconds();
}

void InputCacheNode::update()
{
  const double now_sec = this->now().seconds();
  const auto s = build_snapshot(raw_sensors_, freshness_, thresholds_, now_sec);

  msg::InputSnapshot snapshot_msg;
  snapshot_msg.valid = s.valid;
  snapshot_msg.marker_detected = s.marker_detected;
  snapshot_msg.align_error = s.align_error;
  snapshot_msg.altitude = s.altitude;
  snapshot_msg.delta_h = s.delta_h;
  snapshot_msg.d_horiz = s.d_horiz;
  snapshot_msg.touchdown = s.touchdown;
  snapshot_msg.planner_timeout = s.planner_timeout;
  snapshot_pub_->publish(snapshot_msg);

  msg::TimeoutFlags flags;
  flags.ekf_timeout = s.ekf_timeout || s.tracking_timeout || s.ekf_expired;
  flags.vision_timeout = s.vision_timeout;
  flags.alt_timeout = s.alt_timeout;
  flags.planner_timeout = s.planner_timeout;
  timeout_pub_->publish(flags);

  log_debug(flags, snapshot_msg);
}

void InputCacheNode::log_debug(
  const msg::TimeoutFlags & flags, const msg::InputSnapshot & snapshot) const
{
  if (!debug_enabled_) {
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "valid=%d marker=%d alt=%.2f delta_h=%.2f d_horiz=%.2f ekf_to=%d vis_to=%d alt_to=%d",
    snapshot.valid, snapshot.marker_detected, snapshot.altitude,
    snapshot.delta_h, snapshot.d_horiz,
    flags.ekf_timeout, flags.vision_timeout, flags.alt_timeout);
}

}  // namespace input_state_cache