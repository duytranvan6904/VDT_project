#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <std_msgs/msg/bool.hpp>
#include "offboard_manager/offboard_types.hpp"
#include "offboard_manager/msg/planner_output.hpp"
#include "offboard_manager/msg/offboard_status.hpp"

namespace offboard_manager
{

class OffboardNode : public rclcpp::Node
{
public:
  OffboardNode();

private:
  void on_fsm_state(const std_msgs::msg::UInt8::SharedPtr msg);
  void on_planner_output(const msg::PlannerOutput::SharedPtr msg);
  void on_inhibit(const std_msgs::msg::Bool::SharedPtr msg);
  void publish_status();
  void update();
  void send_heartbeat();
  void publish_setpoint(const Setpoint & sp);
  bool watchdog_check();
  void enter_failsafe();
  void engage_request();
  void publish_vehicle_command(
    uint16_t command, float param1, float param2 = 0.0f, float param3 = 0.0f);
  void log_debug() const;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr fsm_state_sub_;
  rclcpp::Subscription<msg::PlannerOutput>::SharedPtr planner_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr inhibit_sub_;
  rclcpp::Publisher<msg::OffboardStatus>::SharedPtr status_pub_;
  bool inhibited_ = false;

  OffboardContext ctx_;
  FsmState fsm_state_ = FsmState::SEARCH;
  PlannerOutput planner_output_;

  int required_engage_cycles_;
  double watchdog_timeout_sec_;
  float yaw_search_rate_;
  float land_descent_rate_;
  bool debug_enabled_;
};

}  // namespace offboard_manager