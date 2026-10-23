// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vdt_msgs:msg/OffboardStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/offboard_status.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__OFFBOARD_STATUS__BUILDER_HPP_
#define VDT_MSGS__MSG__DETAIL__OFFBOARD_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vdt_msgs/msg/detail/offboard_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vdt_msgs
{

namespace msg
{

namespace builder
{

class Init_OffboardStatus_heartbeat_age_sec
{
public:
  explicit Init_OffboardStatus_heartbeat_age_sec(::vdt_msgs::msg::OffboardStatus & msg)
  : msg_(msg)
  {}
  ::vdt_msgs::msg::OffboardStatus heartbeat_age_sec(::vdt_msgs::msg::OffboardStatus::_heartbeat_age_sec_type arg)
  {
    msg_.heartbeat_age_sec = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vdt_msgs::msg::OffboardStatus msg_;
};

class Init_OffboardStatus_offboard_active
{
public:
  Init_OffboardStatus_offboard_active()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OffboardStatus_heartbeat_age_sec offboard_active(::vdt_msgs::msg::OffboardStatus::_offboard_active_type arg)
  {
    msg_.offboard_active = std::move(arg);
    return Init_OffboardStatus_heartbeat_age_sec(msg_);
  }

private:
  ::vdt_msgs::msg::OffboardStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vdt_msgs::msg::OffboardStatus>()
{
  return vdt_msgs::msg::builder::Init_OffboardStatus_offboard_active();
}

}  // namespace vdt_msgs

#endif  // VDT_MSGS__MSG__DETAIL__OFFBOARD_STATUS__BUILDER_HPP_
