#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include "vdt_msgs/msg/vision_marker.hpp"
#include "vision_interface_bridge/bridge_logic.hpp"

namespace vision_interface_bridge
{

class VisionInterfaceBridge : public rclcpp::Node
{
public:
  VisionInterfaceBridge()
  : Node("vision_interface_bridge")
  {
    image_width_ = declare_parameter<double>("image_width", 640.0);
    image_height_ = declare_parameter<double>("image_height", 480.0);
    fsm_state_timeout_sec_ = declare_parameter<double>("fsm_state_timeout_sec", 1.0);
    const auto detected_topic = declare_parameter<std::string>("detected_topic", "/hpad/detected");
    const auto bbox_topic = declare_parameter<std::string>("bbox_topic", "/hpad/bbox");
    const auto info_topic = declare_parameter<std::string>("camera_info_topic", "/camera_info");
    const auto marker_topic = declare_parameter<std::string>("marker_topic", "vision/marker");
    const auto phase_topic = declare_parameter<std::string>("phase_topic", "/mission/phase");

    rclcpp::QoS sensor_qos(5);
    sensor_qos.best_effort();

    detected_sub_ = create_subscription<std_msgs::msg::Bool>(
      detected_topic, 10,
      std::bind(&VisionInterfaceBridge::on_detected, this, std::placeholders::_1));
    bbox_sub_ = create_subscription<vision_msgs::msg::BoundingBox2D>(
      bbox_topic, 10, std::bind(&VisionInterfaceBridge::on_bbox, this, std::placeholders::_1));
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, sensor_qos,
      std::bind(&VisionInterfaceBridge::on_camera_info, this, std::placeholders::_1));
    fsm_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "fsm/state", 10, std::bind(&VisionInterfaceBridge::on_fsm_state, this, std::placeholders::_1));

    marker_pub_ = create_publisher<vdt_msgs::msg::VisionMarker>(marker_topic, 10);
    phase_pub_ = create_publisher<std_msgs::msg::String>(phase_topic, 10);

    phase_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&VisionInterfaceBridge::publish_phase, this));
  }

private:
  void publish_marker(bool visible, float error)
  {
    vdt_msgs::msg::VisionMarker msg;
    msg.marker_visible = visible;
    msg.pixel_align_error = error;
    marker_pub_->publish(msg);
  }

  void on_detected(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      publish_marker(false, std::nanf(""));
    }
  }

  void on_bbox(const vision_msgs::msg::BoundingBox2D::SharedPtr msg)
  {
    publish_marker(
      true,
      alignment_error(
        msg->center.position.x, msg->center.position.y, image_width_, image_height_));
  }

  void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (msg->width > 0 && msg->height > 0) {
      image_width_ = static_cast<double>(msg->width);
      image_height_ = static_cast<double>(msg->height);
    }
  }

  void on_fsm_state(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    fsm_state_ = msg->data;
    last_fsm_time_ = this->now().seconds();
    has_fsm_ = true;
    publish_phase();
  }

  void publish_phase()
  {
    const bool fresh = has_fsm_ && (this->now().seconds() - last_fsm_time_) <= fsm_state_timeout_sec_;
    std_msgs::msg::String msg;
    msg.data = fresh ? phase_name(fsm_state_) : "IDLE";
    phase_pub_->publish(msg);
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detected_sub_;
  rclcpp::Subscription<vision_msgs::msg::BoundingBox2D>::SharedPtr bbox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr fsm_sub_;
  rclcpp::Publisher<vdt_msgs::msg::VisionMarker>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_pub_;
  rclcpp::TimerBase::SharedPtr phase_timer_;

  double image_width_ = 640.0;
  double image_height_ = 480.0;
  double fsm_state_timeout_sec_ = 1.0;
  bool has_fsm_ = false;
  double last_fsm_time_ = 0.0;
  uint8_t fsm_state_ = 0;
};

}  // namespace vision_interface_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vision_interface_bridge::VisionInterfaceBridge>());
  rclcpp::shutdown();
  return 0;
}