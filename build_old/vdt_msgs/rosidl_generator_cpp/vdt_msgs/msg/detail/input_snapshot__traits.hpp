// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vdt_msgs:msg/InputSnapshot.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/input_snapshot.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__INPUT_SNAPSHOT__TRAITS_HPP_
#define VDT_MSGS__MSG__DETAIL__INPUT_SNAPSHOT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vdt_msgs/msg/detail/input_snapshot__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vdt_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const InputSnapshot & msg,
  std::ostream & out)
{
  out << "{";
  // member: valid
  {
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << ", ";
  }

  // member: marker_detected
  {
    out << "marker_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.marker_detected, out);
    out << ", ";
  }

  // member: align_error
  {
    out << "align_error: ";
    rosidl_generator_traits::value_to_yaml(msg.align_error, out);
    out << ", ";
  }

  // member: altitude
  {
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << ", ";
  }

  // member: delta_h
  {
    out << "delta_h: ";
    rosidl_generator_traits::value_to_yaml(msg.delta_h, out);
    out << ", ";
  }

  // member: d_horiz
  {
    out << "d_horiz: ";
    rosidl_generator_traits::value_to_yaml(msg.d_horiz, out);
    out << ", ";
  }

  // member: touchdown
  {
    out << "touchdown: ";
    rosidl_generator_traits::value_to_yaml(msg.touchdown, out);
    out << ", ";
  }

  // member: planner_timeout
  {
    out << "planner_timeout: ";
    rosidl_generator_traits::value_to_yaml(msg.planner_timeout, out);
    out << ", ";
  }

  // member: yaw_rate
  {
    out << "yaw_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InputSnapshot & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << "\n";
  }

  // member: marker_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "marker_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.marker_detected, out);
    out << "\n";
  }

  // member: align_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "align_error: ";
    rosidl_generator_traits::value_to_yaml(msg.align_error, out);
    out << "\n";
  }

  // member: altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << "\n";
  }

  // member: delta_h
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "delta_h: ";
    rosidl_generator_traits::value_to_yaml(msg.delta_h, out);
    out << "\n";
  }

  // member: d_horiz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d_horiz: ";
    rosidl_generator_traits::value_to_yaml(msg.d_horiz, out);
    out << "\n";
  }

  // member: touchdown
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "touchdown: ";
    rosidl_generator_traits::value_to_yaml(msg.touchdown, out);
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

  // member: yaw_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InputSnapshot & msg, bool use_flow_style = false)
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
  const vdt_msgs::msg::InputSnapshot & msg,
  std::ostream & out, size_t indentation = 0)
{
  vdt_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vdt_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vdt_msgs::msg::InputSnapshot & msg)
{
  return vdt_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vdt_msgs::msg::InputSnapshot>()
{
  return "vdt_msgs::msg::InputSnapshot";
}

template<>
inline const char * name<vdt_msgs::msg::InputSnapshot>()
{
  return "vdt_msgs/msg/InputSnapshot";
}

template<>
struct has_fixed_size<vdt_msgs::msg::InputSnapshot>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vdt_msgs::msg::InputSnapshot>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vdt_msgs::msg::InputSnapshot>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VDT_MSGS__MSG__DETAIL__INPUT_SNAPSHOT__TRAITS_HPP_
