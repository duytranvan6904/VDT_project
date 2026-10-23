// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vdt_msgs:msg/InputSnapshot.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/input_snapshot.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__INPUT_SNAPSHOT__BUILDER_HPP_
#define VDT_MSGS__MSG__DETAIL__INPUT_SNAPSHOT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vdt_msgs/msg/detail/input_snapshot__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vdt_msgs
{

namespace msg
{

namespace builder
{

class Init_InputSnapshot_yaw_rate
{
public:
  explicit Init_InputSnapshot_yaw_rate(::vdt_msgs::msg::InputSnapshot & msg)
  : msg_(msg)
  {}
  ::vdt_msgs::msg::InputSnapshot yaw_rate(::vdt_msgs::msg::InputSnapshot::_yaw_rate_type arg)
  {
    msg_.yaw_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vdt_msgs::msg::InputSnapshot msg_;
};

class Init_InputSnapshot_planner_timeout
{
public:
  explicit Init_InputSnapshot_planner_timeout(::vdt_msgs::msg::InputSnapshot & msg)
  : msg_(msg)
  {}
  Init_InputSnapshot_yaw_rate planner_timeout(::vdt_msgs::msg::InputSnapshot::_planner_timeout_type arg)
  {
    msg_.planner_timeout = std::move(arg);
    return Init_InputSnapshot_yaw_rate(msg_);
  }

private:
  ::vdt_msgs::msg::InputSnapshot msg_;
};

class Init_InputSnapshot_touchdown
{
public:
  explicit Init_InputSnapshot_touchdown(::vdt_msgs::msg::InputSnapshot & msg)
  : msg_(msg)
  {}
  Init_InputSnapshot_planner_timeout touchdown(::vdt_msgs::msg::InputSnapshot::_touchdown_type arg)
  {
    msg_.touchdown = std::move(arg);
    return Init_InputSnapshot_planner_timeout(msg_);
  }

private:
  ::vdt_msgs::msg::InputSnapshot msg_;
};

class Init_InputSnapshot_d_horiz
{
public:
  explicit Init_InputSnapshot_d_horiz(::vdt_msgs::msg::InputSnapshot & msg)
  : msg_(msg)
  {}
  Init_InputSnapshot_touchdown d_horiz(::vdt_msgs::msg::InputSnapshot::_d_horiz_type arg)
  {
    msg_.d_horiz = std::move(arg);
    return Init_InputSnapshot_touchdown(msg_);
  }

private:
  ::vdt_msgs::msg::InputSnapshot msg_;
};

class Init_InputSnapshot_delta_h
{
public:
  explicit Init_InputSnapshot_delta_h(::vdt_msgs::msg::InputSnapshot & msg)
  : msg_(msg)
  {}
  Init_InputSnapshot_d_horiz delta_h(::vdt_msgs::msg::InputSnapshot::_delta_h_type arg)
  {
    msg_.delta_h = std::move(arg);
    return Init_InputSnapshot_d_horiz(msg_);
  }

private:
  ::vdt_msgs::msg::InputSnapshot msg_;
};

class Init_InputSnapshot_altitude
{
public:
  explicit Init_InputSnapshot_altitude(::vdt_msgs::msg::InputSnapshot & msg)
  : msg_(msg)
  {}
  Init_InputSnapshot_delta_h altitude(::vdt_msgs::msg::InputSnapshot::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_InputSnapshot_delta_h(msg_);
  }

private:
  ::vdt_msgs::msg::InputSnapshot msg_;
};

class Init_InputSnapshot_align_error
{
public:
  explicit Init_InputSnapshot_align_error(::vdt_msgs::msg::InputSnapshot & msg)
  : msg_(msg)
  {}
  Init_InputSnapshot_altitude align_error(::vdt_msgs::msg::InputSnapshot::_align_error_type arg)
  {
    msg_.align_error = std::move(arg);
    return Init_InputSnapshot_altitude(msg_);
  }

private:
  ::vdt_msgs::msg::InputSnapshot msg_;
};

class Init_InputSnapshot_marker_detected
{
public:
  explicit Init_InputSnapshot_marker_detected(::vdt_msgs::msg::InputSnapshot & msg)
  : msg_(msg)
  {}
  Init_InputSnapshot_align_error marker_detected(::vdt_msgs::msg::InputSnapshot::_marker_detected_type arg)
  {
    msg_.marker_detected = std::move(arg);
    return Init_InputSnapshot_align_error(msg_);
  }

private:
  ::vdt_msgs::msg::InputSnapshot msg_;
};

class Init_InputSnapshot_valid
{
public:
  Init_InputSnapshot_valid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InputSnapshot_marker_detected valid(::vdt_msgs::msg::InputSnapshot::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return Init_InputSnapshot_marker_detected(msg_);
  }

private:
  ::vdt_msgs::msg::InputSnapshot msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vdt_msgs::msg::InputSnapshot>()
{
  return vdt_msgs::msg::builder::Init_InputSnapshot_valid();
}

}  // namespace vdt_msgs

#endif  // VDT_MSGS__MSG__DETAIL__INPUT_SNAPSHOT__BUILDER_HPP_
