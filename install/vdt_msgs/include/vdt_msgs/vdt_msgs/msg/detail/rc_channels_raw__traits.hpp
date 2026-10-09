// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vdt_msgs:msg/RcChannelsRaw.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/rc_channels_raw.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__RC_CHANNELS_RAW__TRAITS_HPP_
#define VDT_MSGS__MSG__DETAIL__RC_CHANNELS_RAW__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vdt_msgs/msg/detail/rc_channels_raw__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vdt_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RcChannelsRaw & msg,
  std::ostream & out)
{
  out << "{";
  // member: ch
  {
    if (msg.ch.size() == 0) {
      out << "ch: []";
    } else {
      out << "ch: [";
      size_t pending_items = msg.ch.size();
      for (auto item : msg.ch) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: valid
  {
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << ", ";
  }

  // member: failsafe
  {
    out << "failsafe: ";
    rosidl_generator_traits::value_to_yaml(msg.failsafe, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RcChannelsRaw & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ch.size() == 0) {
      out << "ch: []\n";
    } else {
      out << "ch:\n";
      for (auto item : msg.ch) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << "\n";
  }

  // member: failsafe
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "failsafe: ";
    rosidl_generator_traits::value_to_yaml(msg.failsafe, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RcChannelsRaw & msg, bool use_flow_style = false)
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
  const vdt_msgs::msg::RcChannelsRaw & msg,
  std::ostream & out, size_t indentation = 0)
{
  vdt_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vdt_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vdt_msgs::msg::RcChannelsRaw & msg)
{
  return vdt_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vdt_msgs::msg::RcChannelsRaw>()
{
  return "vdt_msgs::msg::RcChannelsRaw";
}

template<>
inline const char * name<vdt_msgs::msg::RcChannelsRaw>()
{
  return "vdt_msgs/msg/RcChannelsRaw";
}

template<>
struct has_fixed_size<vdt_msgs::msg::RcChannelsRaw>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vdt_msgs::msg::RcChannelsRaw>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vdt_msgs::msg::RcChannelsRaw>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VDT_MSGS__MSG__DETAIL__RC_CHANNELS_RAW__TRAITS_HPP_
