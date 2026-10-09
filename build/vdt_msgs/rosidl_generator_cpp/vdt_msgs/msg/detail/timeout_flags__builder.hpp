// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vdt_msgs:msg/TimeoutFlags.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/timeout_flags.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__TIMEOUT_FLAGS__BUILDER_HPP_
#define VDT_MSGS__MSG__DETAIL__TIMEOUT_FLAGS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vdt_msgs/msg/detail/timeout_flags__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vdt_msgs
{

namespace msg
{

namespace builder
{

class Init_TimeoutFlags_planner_timeout
{
public:
  explicit Init_TimeoutFlags_planner_timeout(::vdt_msgs::msg::TimeoutFlags & msg)
  : msg_(msg)
  {}
  ::vdt_msgs::msg::TimeoutFlags planner_timeout(::vdt_msgs::msg::TimeoutFlags::_planner_timeout_type arg)
  {
    msg_.planner_timeout = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vdt_msgs::msg::TimeoutFlags msg_;
};

class Init_TimeoutFlags_alt_timeout
{
public:
  explicit Init_TimeoutFlags_alt_timeout(::vdt_msgs::msg::TimeoutFlags & msg)
  : msg_(msg)
  {}
  Init_TimeoutFlags_planner_timeout alt_timeout(::vdt_msgs::msg::TimeoutFlags::_alt_timeout_type arg)
  {
    msg_.alt_timeout = std::move(arg);
    return Init_TimeoutFlags_planner_timeout(msg_);
  }

private:
  ::vdt_msgs::msg::TimeoutFlags msg_;
};

class Init_TimeoutFlags_vision_timeout
{
public:
  explicit Init_TimeoutFlags_vision_timeout(::vdt_msgs::msg::TimeoutFlags & msg)
  : msg_(msg)
  {}
  Init_TimeoutFlags_alt_timeout vision_timeout(::vdt_msgs::msg::TimeoutFlags::_vision_timeout_type arg)
  {
    msg_.vision_timeout = std::move(arg);
    return Init_TimeoutFlags_alt_timeout(msg_);
  }

private:
  ::vdt_msgs::msg::TimeoutFlags msg_;
};

class Init_TimeoutFlags_ekf_timeout
{
public:
  Init_TimeoutFlags_ekf_timeout()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TimeoutFlags_vision_timeout ekf_timeout(::vdt_msgs::msg::TimeoutFlags::_ekf_timeout_type arg)
  {
    msg_.ekf_timeout = std::move(arg);
    return Init_TimeoutFlags_vision_timeout(msg_);
  }

private:
  ::vdt_msgs::msg::TimeoutFlags msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vdt_msgs::msg::TimeoutFlags>()
{
  return vdt_msgs::msg::builder::Init_TimeoutFlags_ekf_timeout();
}

}  // namespace vdt_msgs

#endif  // VDT_MSGS__MSG__DETAIL__TIMEOUT_FLAGS__BUILDER_HPP_
