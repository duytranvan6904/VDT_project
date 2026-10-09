// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vdt_msgs:msg/TimeoutFlags.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/timeout_flags.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__TIMEOUT_FLAGS__TRAITS_HPP_
#define VDT_MSGS__MSG__DETAIL__TIMEOUT_FLAGS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vdt_msgs/msg/detail/timeout_flags__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vdt_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TimeoutFlags & msg,
  std::ostream & out)
{
  out << "{";
  // member: ekf_timeout
  {
    out << "ekf_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.ekf_timeout, out);
    out << ", ";
  }

  // member: vision_timeout
  {
    out << "vision_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.vision_timeout, out);
    out << ", ";
  }

  // member: alt_timeout
  {
    out << "alt_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.alt_timeout, out);
    out << ", ";
  }

  // member: planner_timeout
  {
    out << "planner_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.planner_timeout, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TimeoutFlags & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ekf_timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ekf_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.ekf_timeout, out);
    out << "\n";
  }

  // member: vision_timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vision_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.vision_timeout, out);
    out << "\n";
  }

  // member: alt_timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alt_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.alt_timeout, out);
    out << "\n";
  }

  // member: planner_timeout
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "planner_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.planner_timeout, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TimeoutFlags & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace vdt_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vdt_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vdt_msgs::msg::TimeoutFlags & msg,
  std::ostream & out, size_t indentation = 0)
{
  vdt_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vdt_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vdt_msgs::msg::TimeoutFlags & msg)
{
  return vdt_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vdt_msgs::msg::TimeoutFlags>()
{
  return "vdt_msgs::msg::TimeoutFlags";
}

template<>
inline const char * name<vdt_msgs::msg::TimeoutFlags>()
{
  return "vdt_msgs/msg/TimeoutFlags";
}

template<>
struct has_fixed_size<vdt_msgs::msg::TimeoutFlags>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vdt_msgs::msg::TimeoutFlags>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vdt_msgs::msg::TimeoutFlags>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VDT_MSGS__MSG__DETAIL__TIMEOUT_FLAGS__TRAITS_HPP_
