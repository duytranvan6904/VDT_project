// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vdt_msgs:msg/RcChannelsRaw.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/rc_channels_raw.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__RC_CHANNELS_RAW__BUILDER_HPP_
#define VDT_MSGS__MSG__DETAIL__RC_CHANNELS_RAW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vdt_msgs/msg/detail/rc_channels_raw__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vdt_msgs
{

namespace msg
{

namespace builder
{

class Init_RcChannelsRaw_failsafe
{
public:
  explicit Init_RcChannelsRaw_failsafe(::vdt_msgs::msg::RcChannelsRaw & msg)
  : msg_(msg)
  {}
  ::vdt_msgs::msg::RcChannelsRaw failsafe(::vdt_msgs::msg::RcChannelsRaw::_failsafe_type arg)
  {
    msg_.failsafe = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vdt_msgs::msg::RcChannelsRaw msg_;
};

class Init_RcChannelsRaw_valid
{
public:
  explicit Init_RcChannelsRaw_valid(::vdt_msgs::msg::RcChannelsRaw & msg)
  : msg_(msg)
  {}
  Init_RcChannelsRaw_failsafe valid(::vdt_msgs::msg::RcChannelsRaw::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return Init_RcChannelsRaw_failsafe(msg_);
  }

private:
  ::vdt_msgs::msg::RcChannelsRaw msg_;
};

class Init_RcChannelsRaw_ch
{
public:
  Init_RcChannelsRaw_ch()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RcChannelsRaw_valid ch(::vdt_msgs::msg::RcChannelsRaw::_ch_type arg)
  {
    msg_.ch = std::move(arg);
    return Init_RcChannelsRaw_valid(msg_);
  }

private:
  ::vdt_msgs::msg::RcChannelsRaw msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vdt_msgs::msg::RcChannelsRaw>()
{
  return vdt_msgs::msg::builder::Init_RcChannelsRaw_ch();
}

}  // namespace vdt_msgs

#endif  // VDT_MSGS__MSG__DETAIL__RC_CHANNELS_RAW__BUILDER_HPP_
