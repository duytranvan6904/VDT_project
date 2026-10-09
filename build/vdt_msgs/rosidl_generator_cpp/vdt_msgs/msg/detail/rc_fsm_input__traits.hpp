// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vdt_msgs:msg/RcFsmInput.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/rc_fsm_input.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__RC_FSM_INPUT__TRAITS_HPP_
#define VDT_MSGS__MSG__DETAIL__RC_FSM_INPUT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vdt_msgs/msg/detail/rc_fsm_input__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vdt_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RcFsmInput & msg,
  std::ostream & out)
{
  out << "{";
  // member: land_switch
  {
    out << "land_switch: ";
    rosidl_generator_traits::value_to_yaml(msg.land_switch, out);
    out << ", ";
  }

  // member: kill_switch
  {
    out << "kill_switch: ";
    rosidl_generator_traits::value_to_yaml(msg.kill_switch, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RcFsmInput & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: land_switch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "land_switch: ";
    rosidl_generator_traits::value_to_yaml(msg.land_switch, out);
    out << "\n";
  }

  // member: kill_switch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kill_switch: ";
    rosidl_generator_traits::value_to_yaml(msg.kill_switch, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RcFsmInput & msg, bool use_flow_style = false)
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
  const vdt_msgs::msg::RcFsmInput & msg,
  std::ostream & out, size_t indentation = 0)
{
  vdt_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vdt_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vdt_msgs::msg::RcFsmInput & msg)
{
  return vdt_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vdt_msgs::msg::RcFsmInput>()
{
  return "vdt_msgs::msg::RcFsmInput";
}

template<>
inline const char * name<vdt_msgs::msg::RcFsmInput>()
{
  return "vdt_msgs/msg/RcFsmInput";
}

template<>
struct has_fixed_size<vdt_msgs::msg::RcFsmInput>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vdt_msgs::msg::RcFsmInput>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vdt_msgs::msg::RcFsmInput>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VDT_MSGS__MSG__DETAIL__RC_FSM_INPUT__TRAITS_HPP_
