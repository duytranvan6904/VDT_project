// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vdt_msgs:msg/AltEstimate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/alt_estimate.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__ALT_ESTIMATE__BUILDER_HPP_
#define VDT_MSGS__MSG__DETAIL__ALT_ESTIMATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vdt_msgs/msg/detail/alt_estimate__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vdt_msgs
{

namespace msg
{

namespace builder
{

class Init_AltEstimate_touchdown_flag
{
public:
  explicit Init_AltEstimate_touchdown_flag(::vdt_msgs::msg::AltEstimate & msg)
  : msg_(msg)
  {}
  ::vdt_msgs::msg::AltEstimate touchdown_flag(::vdt_msgs::msg::AltEstimate::_touchdown_flag_type arg)
  {
    msg_.touchdown_flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vdt_msgs::msg::AltEstimate msg_;
};

class Init_AltEstimate_altitude
{
public:
  Init_AltEstimate_altitude()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AltEstimate_touchdown_flag altitude(::vdt_msgs::msg::AltEstimate::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_AltEstimate_touchdown_flag(msg_);
  }

private:
  ::vdt_msgs::msg::AltEstimate msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vdt_msgs::msg::AltEstimate>()
{
  return vdt_msgs::msg::builder::Init_AltEstimate_altitude();
}

}  // namespace vdt_msgs

#endif  // VDT_MSGS__MSG__DETAIL__ALT_ESTIMATE__BUILDER_HPP_
