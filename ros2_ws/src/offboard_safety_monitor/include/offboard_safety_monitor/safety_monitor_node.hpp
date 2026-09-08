#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include "offboard_manager/msg/offboard_status.hpp"
#include "offboard_safety_monitor/safety_types.hpp"

namespace offboard_safety_monitor
{

class SafetyMonitorNode : public rclcpp::Node
{
public:
  SafetyMonitorNode();

private:
  void on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void on_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void on_battery(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
  void on_offboard_status(const offboard_manager::msg::OffboardStatus::SharedPtr msg);

  void update();
  void execute_action(FailsafeLevel level);
  void publish_mode_command(uint8_t main_mode, uint8_t sub_mode);
  void publish_inhibit(bool inhibit);
  void publish_force_land(bool force_land);
  void log_debug(FailsafeLevel level) const;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<offboard_manager::msg::OffboardStatus>::SharedPtr offboard_status_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr inhibit_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr force_land_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  px4_msgs::msg::VehicleStatus vehicle_status_;
  px4_msgs::msg::VehicleLocalPosition local_position_;
  px4_msgs::msg::BatteryStatus battery_status_;
  offboard_manager::msg::OffboardStatus offboard_status_;

  SafetyThresholds thresholds_;
  SafetyContext ctx_;
  bool debug_enabled_;
};

}  // namespace offboard_safety_monitor