// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vdt_msgs:msg/OffboardStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/offboard_status.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__OFFBOARD_STATUS__TRAITS_HPP_
#define VDT_MSGS__MSG__DETAIL__OFFBOARD_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vdt_msgs/msg/detail/offboard_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vdt_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OffboardStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: offboard_active
  {
    out << "offboard_active: ";
    rosidl_generator_traits::value_to_yaml(msg.offboard_active, out);
    out << ", ";
  }

  // member: heartbeat_age_sec
  {
    out << "heartbeat_age_sec: ";
    rosidl_generator_traits::value_to_yaml(msg.heartbeat_age_sec, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OffboardStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: offboard_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "offboard_active: ";
    rosidl_generator_traits::value_to_yaml(msg.offboard_active, out);
    out << "\n";
  }

  // member: heartbeat_age_sec
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heartbeat_age_sec: ";
    rosidl_generator_traits::value_to_yaml(msg.heartbeat_age_sec, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OffboardStatus & msg, bool use_flow_style = false)
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
  const vdt_msgs::msg::OffboardStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  vdt_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vdt_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vdt_msgs::msg::OffboardStatus & msg)
{
  return vdt_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vdt_msgs::msg::OffboardStatus>()
{
  return "vdt_msgs::msg::OffboardStatus";
}

template<>
inline const char * name<vdt_msgs::msg::OffboardStatus>()
{
  return "vdt_msgs/msg/OffboardStatus";
}

template<>
struct has_fixed_size<vdt_msgs::msg::OffboardStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vdt_msgs::msg::OffboardStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vdt_msgs::msg::OffboardStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VDT_MSGS__MSG__DETAIL__OFFBOARD_STATUS__TRAITS_HPP_
