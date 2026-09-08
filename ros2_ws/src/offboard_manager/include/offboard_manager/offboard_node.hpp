#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include "offboard_manager/offboard_types.hpp"
#include "offboard_manager/msg/planner_output.hpp"
#include "offboard_manager/msg/offboard_status.hpp"
#include "fsm_state_machine/msg/timeout_flags.hpp"

namespace offboard_manager
{

class OffboardNode : public rclcpp::Node
{
public:
  OffboardNode();

private:
  void on_fsm_state(const std_msgs::msg::UInt8::SharedPtr msg);
  void on_planner_output(const msg::PlannerOutput::SharedPtr msg);
  void on_killed(const std_msgs::msg::Bool::SharedPtr msg);
  void on_inhibit(const std_msgs::msg::Bool::SharedPtr msg);
  void on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void on_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void on_timeout_flags(const fsm_state_machine::msg::TimeoutFlags::SharedPtr msg);

  void update();
  void reset_engage_sequence();
  void send_heartbeat();
  void publish_setpoint(const Setpoint & sp);
  bool watchdog_check();
  void enter_failsafe();
  void engage_request();
  bool is_vehicle_status_fresh() const;
  bool is_local_position_fresh() const;
  bool is_px4_ready() const;
  void publish_vehicle_command(
    uint16_t command, float param1, float param2 = 0.0f, float param3 = 0.0f);
  void publish_status();
  void log_debug() const;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr fsm_state_sub_;
  rclcpp::Subscription<msg::PlannerOutput>::SharedPtr planner_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr killed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr inhibit_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<msg::OffboardStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<fsm_state_machine::msg::TimeoutFlags>::SharedPtr timeout_sub_;
  bool planner_timeout_ = false; 

  OffboardContext ctx_;
  FsmState fsm_state_ = FsmState::SEARCH;
  PlannerOutput planner_output_;

  px4_msgs::msg::VehicleStatus vehicle_status_;
  px4_msgs::msg::VehicleLocalPosition local_position_;
  bool has_vehicle_status_ = false;
  bool has_local_position_ = false;
  double last_vehicle_status_time_ = 0.0;
  double last_local_position_time_ = 0.0;
  bool killed_ = false;
  bool inhibited_ = false;

  EngagePhase engage_phase_ = EngagePhase::WAIT_SETPOINT_STREAM;
  int phase_cycle_count_ = 0;

  int required_engage_cycles_;
  int mode_confirm_timeout_cycles_;
  int health_confirm_timeout_cycles_;
  double data_freshness_timeout_sec_;
  float max_horizontal_velocity_;
  float max_vertical_velocity_;
  float max_yaw_;
  double watchdog_timeout_sec_;
  float yaw_search_rate_;
  float land_descent_rate_;
  bool debug_enabled_;
};

}  // namespace offboard_manager