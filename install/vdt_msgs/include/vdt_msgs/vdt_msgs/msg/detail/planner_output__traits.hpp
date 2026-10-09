// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vdt_msgs:msg/PlannerOutput.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/planner_output.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__PLANNER_OUTPUT__TRAITS_HPP_
#define VDT_MSGS__MSG__DETAIL__PLANNER_OUTPUT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vdt_msgs/msg/detail/planner_output__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vdt_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PlannerOutput & msg,
  std::ostream & out)
{
  out << "{";
  // member: vx
  {
    out << "vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vx, out);
    out << ", ";
  }

  // member: vy
  {
    out << "vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vy, out);
    out << ", ";
  }

  // member: vz
  {
    out << "vz: ";
    rosidl_generator_traits::value_to_yaml(msg.vz, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlannerOutput & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: vx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vx, out);
    out << "\n";
  }

  // member: vy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vy, out);
    out << "\n";
  }

  // member: vz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vz: ";
    rosidl_generator_traits::value_to_yaml(msg.vz, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlannerOutput & msg, bool use_flow_style = false)
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
  const vdt_msgs::msg::PlannerOutput & msg,
  std::ostream & out, size_t indentation = 0)
{
  vdt_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vdt_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vdt_msgs::msg::PlannerOutput & msg)
{
  return vdt_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vdt_msgs::msg::PlannerOutput>()
{
  return "vdt_msgs::msg::PlannerOutput";
}

template<>
inline const char * name<vdt_msgs::msg::PlannerOutput>()
{
  return "vdt_msgs/msg/PlannerOutput";
}

template<>
struct has_fixed_size<vdt_msgs::msg::PlannerOutput>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vdt_msgs::msg::PlannerOutput>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vdt_msgs::msg::PlannerOutput>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VDT_MSGS__MSG__DETAIL__PLANNER_OUTPUT__TRAITS_HPP_
