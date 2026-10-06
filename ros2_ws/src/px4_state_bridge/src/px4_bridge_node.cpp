#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "fsm_state_machine/msg/alt_estimate.hpp"
#include "px4_state_bridge/frame_conversion.hpp"

namespace px4_state_bridge
{

namespace
{
constexpr uint8_t kFrameNed = 1;

void fill_tf(
  geometry_msgs::msg::TransformStamped & tf, const rclcpp::Time & stamp,
  const std::string & parent, const std::string & child, const Vec3 & t, const Quat & q)
{
  tf.header.stamp = stamp;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.transform.translation.x = t[0];
  tf.transform.translation.y = t[1];
  tf.transform.translation.z = t[2];
  tf.transform.rotation.w = q[0];
  tf.transform.rotation.x = q[1];
  tf.transform.rotation.y = q[2];
  tf.transform.rotation.z = q[3];
}

bool all_finite(const float * v, size_t n)
{
  for (size_t i = 0; i < n; ++i) {
    if (!std::isfinite(v[i])) {
      return false;
    }
  }
  return true;
}
}  // namespace

class Px4StateBridge : public rclcpp::Node
{
public:
  Px4StateBridge()
  : Node("px4_state_bridge")
  {
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    gimbal_frame_ = declare_parameter<std::string>("gimbal_frame", "gimbal_link");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_optical_frame");
    pivot_ = to_vec3(
      "gimbal_pivot_xyz",
      declare_parameter<std::vector<double>>("gimbal_pivot_xyz", {0.0, 0.0, 0.0}));
    camera_in_gimbal_ = to_vec3(
      "camera_in_gimbal_xyz",
      declare_parameter<std::vector<double>>("camera_in_gimbal_xyz", {0.0, 0.0, 0.0}));
    use_ground_contact_ = declare_parameter<bool>("use_ground_contact", false);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.5);
    const auto odometry_topic =
      declare_parameter<std::string>("odometry_topic", "/fmu/out/vehicle_odometry");
    const auto land_topic =
      declare_parameter<std::string>("land_detected_topic", "/fmu/out/vehicle_land_detected");
    const auto gimbal_topic =
      declare_parameter<std::string>("gimbal_angle_topic", "gimbal/target_angle_deg");

    rclcpp::QoS px4_qos(1);
    px4_qos.best_effort();

    odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      odometry_topic, px4_qos,
      std::bind(&Px4StateBridge::on_odometry, this, std::placeholders::_1));
    land_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      land_topic, px4_qos,
      std::bind(&Px4StateBridge::on_land_detected, this, std::placeholders::_1));
    gimbal_sub_ = create_subscription<std_msgs::msg::Float32>(
      gimbal_topic, 10, std::bind(&Px4StateBridge::on_gimbal, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    alt_pub_ = create_publisher<fsm_state_machine::msg::AltEstimate>("alt_estimator/state", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    alt_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&Px4StateBridge::publish_alt, this));
  }

private:
  Vec3 to_vec3(const std::string & name, const std::vector<double> & v)
  {
    if (v.size() != 3) {
      RCLCPP_WARN(get_logger(), "%s needs 3 values, using zeros", name.c_str());
      return {0.0, 0.0, 0.0};
    }
    return {v[0], v[1], v[2]};
  }

  void on_gimbal(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (std::isfinite(msg->data)) {
      gimbal_deg_ = msg->data;
    }
  }

  void on_land_detected(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
  {
    landed_ = msg->landed;
    ground_contact_ = msg->ground_contact;
    land_time_ = this->now().seconds();
    has_land_ = true;
  }

  void on_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    if (msg->pose_frame != kFrameNed) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "vehicle_odometry pose_frame %u is not NED, dropped",
        static_cast<unsigned>(msg->pose_frame));
      return;
    }
    if (!all_finite(msg->position.data(), 3) || !all_finite(msg->q.data(), 4)) {
      return;
    }
    const Quat q_ned_frd = {msg->q[0], msg->q[1], msg->q[2], msg->q[3]};
    if (quat_norm(q_ned_frd) < 0.5) {
      return;
    }

    const rclcpp::Time stamp = this->now();
    const Vec3 p = ned_to_enu({msg->position[0], msg->position[1], msg->position[2]});
    const Quat q = frd_ned_to_flu_enu(q_ned_frd);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = p[0];
    odom.pose.pose.position.y = p[1];
    odom.pose.pose.position.z = p[2];
    odom.pose.pose.orientation.w = q[0];
    odom.pose.pose.orientation.x = q[1];
    odom.pose.pose.orientation.y = q[2];
    odom.pose.pose.orientation.z = q[3];
    if (msg->velocity_frame == kFrameNed && all_finite(msg->velocity.data(), 3)) {
      const Vec3 v = ned_to_enu({msg->velocity[0], msg->velocity[1], msg->velocity[2]});
      odom.twist.twist.linear.x = v[0];
      odom.twist.twist.linear.y = v[1];
      odom.twist.twist.linear.z = v[2];
    }
    if (all_finite(msg->angular_velocity.data(), 3)) {
      const Vec3 w = frd_to_flu(
        {msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]});
      odom.twist.twist.angular.x = w[0];
      odom.twist.twist.angular.y = w[1];
      odom.twist.twist.angular.z = w[2];
    }
    odom_pub_->publish(odom);

    std::vector<geometry_msgs::msg::TransformStamped> tfs(3);
    fill_tf(tfs[0], stamp, world_frame_, base_frame_, p, q);
    fill_tf(tfs[1], stamp, base_frame_, gimbal_frame_, pivot_, gimbal_rotation(gimbal_deg_));
    fill_tf(tfs[2], stamp, gimbal_frame_, camera_frame_, camera_in_gimbal_, optical_from_link());
    tf_broadcaster_->sendTransform(tfs);

    altitude_ = p[2];
    odom_time_ = stamp.seconds();
    has_odom_ = true;
  }

  void publish_alt()
  {
    const double now_sec = this->now().seconds();
    if (!has_odom_ || (now_sec - odom_time_) > odom_timeout_sec_) {
      return;
    }
    const bool land_fresh = has_land_ && (now_sec - land_time_) <= 1.0;
    fsm_state_machine::msg::AltEstimate msg;
    msg.altitude = static_cast<float>(altitude_);
    msg.touchdown_flag = land_fresh && (landed_ || (use_ground_contact_ && ground_contact_));
    alt_pub_->publish(msg);
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gimbal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<fsm_state_machine::msg::AltEstimate>::SharedPtr alt_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr alt_timer_;

  std::string world_frame_, base_frame_, gimbal_frame_, camera_frame_;
  Vec3 pivot_{0.0, 0.0, 0.0};
  Vec3 camera_in_gimbal_{0.0, 0.0, 0.0};
  bool use_ground_contact_ = false;
  double odom_timeout_sec_ = 0.5;

  bool has_odom_ = false;
  double odom_time_ = 0.0;
  double altitude_ = 0.0;
  bool has_land_ = false;
  double land_time_ = 0.0;
  bool landed_ = false;
  bool ground_contact_ = false;
  double gimbal_deg_ = 0.0;
};

}  // namespace px4_state_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_state_bridge::Px4StateBridge>());
  rclcpp::shutdown();
  return 0;
}