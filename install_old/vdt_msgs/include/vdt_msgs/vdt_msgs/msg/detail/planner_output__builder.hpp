// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vdt_msgs:msg/PlannerOutput.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/planner_output.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__PLANNER_OUTPUT__BUILDER_HPP_
#define VDT_MSGS__MSG__DETAIL__PLANNER_OUTPUT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vdt_msgs/msg/detail/planner_output__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vdt_msgs
{

namespace msg
{

namespace builder
{

class Init_PlannerOutput_yaw
{
public:
  explicit Init_PlannerOutput_yaw(::vdt_msgs::msg::PlannerOutput & msg)
  : msg_(msg)
  {}
  ::vdt_msgs::msg::PlannerOutput yaw(::vdt_msgs::msg::PlannerOutput::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vdt_msgs::msg::PlannerOutput msg_;
};

class Init_PlannerOutput_vz
{
public:
  explicit Init_PlannerOutput_vz(::vdt_msgs::msg::PlannerOutput & msg)
  : msg_(msg)
  {}
  Init_PlannerOutput_yaw vz(::vdt_msgs::msg::PlannerOutput::_vz_type arg)
  {
    msg_.vz = std::move(arg);
    return Init_PlannerOutput_yaw(msg_);
  }

private:
  ::vdt_msgs::msg::PlannerOutput msg_;
};

class Init_PlannerOutput_vy
{
public:
  explicit Init_PlannerOutput_vy(::vdt_msgs::msg::PlannerOutput & msg)
  : msg_(msg)
  {}
  Init_PlannerOutput_vz vy(::vdt_msgs::msg::PlannerOutput::_vy_type arg)
  {
    msg_.vy = std::move(arg);
    return Init_PlannerOutput_vz(msg_);
  }

private:
  ::vdt_msgs::msg::PlannerOutput msg_;
};

class Init_PlannerOutput_vx
{
public:
  Init_PlannerOutput_vx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlannerOutput_vy vx(::vdt_msgs::msg::PlannerOutput::_vx_type arg)
  {
    msg_.vx = std::move(arg);
    return Init_PlannerOutput_vy(msg_);
  }

private:
  ::vdt_msgs::msg::PlannerOutput msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vdt_msgs::msg::PlannerOutput>()
{
  return vdt_msgs::msg::builder::Init_PlannerOutput_vx();
}

}  // namespace vdt_msgs

#endif  // VDT_MSGS__MSG__DETAIL__PLANNER_OUTPUT__BUILDER_HPP_
