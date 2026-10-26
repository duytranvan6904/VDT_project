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
#include "vdt_msgs/msg/planner_output.hpp"
#include "vdt_msgs/msg/offboard_status.hpp"
#include "vdt_msgs/msg/timeout_flags.hpp"

namespace offboard_manager
{
namespace msg = vdt_msgs::msg;
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
  void on_timeout_flags(const vdt_msgs::msg::TimeoutFlags::SharedPtr msg);
  void on_touchdown(const std_msgs::msg::Bool::SharedPtr msg);
  void on_disarm_request(const std_msgs::msg::Bool::SharedPtr msg);
  void send_force_disarm();

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
  bool is_airborne() const;
  bool is_ready_to_engage() const;
  bool is_planner_stale() const;
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
  rclcpp::Subscription<vdt_msgs::msg::TimeoutFlags>::SharedPtr timeout_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr touchdown_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr disarm_request_sub_;
  bool planner_timeout_ = true;

  OffboardContext ctx_;
  FsmState fsm_state_ = FsmState::SEARCH;
  PlannerOutput planner_output_;
  bool has_planner_ = false;
  double last_planner_time_ = 0.0;

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
  bool arm_command_sent_ = false;

  int required_engage_cycles_;
  int mode_confirm_timeout_cycles_;
  int health_confirm_timeout_cycles_;
  double data_freshness_timeout_sec_;
  float max_horizontal_velocity_;
  float max_vertical_velocity_; 
  float max_yaw_;
  double watchdog_timeout_sec_;
  double planner_timeout_sec_;
  float min_engage_altitude_m_;
  float yaw_search_rate_;
  float land_descent_rate_;
  bool debug_enabled_;
  bool require_loiter_before_engage_;
  bool is_airborne_latched_ = false;
  double last_valid_alt_m_ = 0.0;
  bool disarming_active_ = false;
  double disarm_start_time_ = 0.0;
  double disarm_hold_duration_sec_ = 0.35;
  float disarm_press_down_vz_ = 0.12f;
};

}  // namespace offboard_manager