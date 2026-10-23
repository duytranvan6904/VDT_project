// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vdt_msgs:msg/RcFsmInput.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/rc_fsm_input.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__RC_FSM_INPUT__BUILDER_HPP_
#define VDT_MSGS__MSG__DETAIL__RC_FSM_INPUT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vdt_msgs/msg/detail/rc_fsm_input__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vdt_msgs
{

namespace msg
{

namespace builder
{

class Init_RcFsmInput_kill_switch
{
public:
  explicit Init_RcFsmInput_kill_switch(::vdt_msgs::msg::RcFsmInput & msg)
  : msg_(msg)
  {}
  ::vdt_msgs::msg::RcFsmInput kill_switch(::vdt_msgs::msg::RcFsmInput::_kill_switch_type arg)
  {
    msg_.kill_switch = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vdt_msgs::msg::RcFsmInput msg_;
};

class Init_RcFsmInput_land_switch
{
public:
  Init_RcFsmInput_land_switch()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RcFsmInput_kill_switch land_switch(::vdt_msgs::msg::RcFsmInput::_land_switch_type arg)
  {
    msg_.land_switch = std::move(arg);
    return Init_RcFsmInput_kill_switch(msg_);
  }

private:
  ::vdt_msgs::msg::RcFsmInput msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vdt_msgs::msg::RcFsmInput>()
{
  return vdt_msgs::msg::builder::Init_RcFsmInput_land_switch();
}

}  // namespace vdt_msgs

#endif  // VDT_MSGS__MSG__DETAIL__RC_FSM_INPUT__BUILDER_HPP_
