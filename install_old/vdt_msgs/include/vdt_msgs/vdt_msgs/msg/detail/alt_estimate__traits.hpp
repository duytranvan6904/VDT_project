// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vdt_msgs:msg/AltEstimate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/alt_estimate.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__ALT_ESTIMATE__TRAITS_HPP_
#define VDT_MSGS__MSG__DETAIL__ALT_ESTIMATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vdt_msgs/msg/detail/alt_estimate__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vdt_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AltEstimate & msg,
  std::ostream & out)
{
  out << "{";
  // member: altitude
  {
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << ", ";
  }

  // member: touchdown_flag
  {
    out << "touchdown_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.touchdown_flag, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AltEstimate & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << "\n";
  }

  // member: touchdown_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "touchdown_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.touchdown_flag, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AltEstimate & msg, bool use_flow_style = false)
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
  const vdt_msgs::msg::AltEstimate & msg,
  std::ostream & out, size_t indentation = 0)
{
  vdt_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vdt_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vdt_msgs::msg::AltEstimate & msg)
{
  return vdt_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vdt_msgs::msg::AltEstimate>()
{
  return "vdt_msgs::msg::AltEstimate";
}

template<>
inline const char * name<vdt_msgs::msg::AltEstimate>()
{
  return "vdt_msgs/msg/AltEstimate";
}

template<>
struct has_fixed_size<vdt_msgs::msg::AltEstimate>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vdt_msgs::msg::AltEstimate>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vdt_msgs::msg::AltEstimate>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VDT_MSGS__MSG__DETAIL__ALT_ESTIMATE__TRAITS_HPP_
