// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vdt_msgs:msg/VisionMarker.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/vision_marker.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__VISION_MARKER__TRAITS_HPP_
#define VDT_MSGS__MSG__DETAIL__VISION_MARKER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vdt_msgs/msg/detail/vision_marker__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vdt_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const VisionMarker & msg,
  std::ostream & out)
{
  out << "{";
  // member: marker_visible
  {
    out << "marker_visible: ";
    rosidl_generator_traits::value_to_yaml(msg.marker_visible, out);
    out << ", ";
  }

  // member: pixel_align_error
  {
    out << "pixel_align_error: ";
    rosidl_generator_traits::value_to_yaml(msg.pixel_align_error, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VisionMarker & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: marker_visible
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "marker_visible: ";
    rosidl_generator_traits::value_to_yaml(msg.marker_visible, out);
    out << "\n";
  }

  // member: pixel_align_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pixel_align_error: ";
    rosidl_generator_traits::value_to_yaml(msg.pixel_align_error, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VisionMarker & msg, bool use_flow_style = false)
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
  const vdt_msgs::msg::VisionMarker & msg,
  std::ostream & out, size_t indentation = 0)
{
  vdt_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vdt_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vdt_msgs::msg::VisionMarker & msg)
{
  return vdt_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vdt_msgs::msg::VisionMarker>()
{
  return "vdt_msgs::msg::VisionMarker";
}

template<>
inline const char * name<vdt_msgs::msg::VisionMarker>()
{
  return "vdt_msgs/msg/VisionMarker";
}

template<>
struct has_fixed_size<vdt_msgs::msg::VisionMarker>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vdt_msgs::msg::VisionMarker>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vdt_msgs::msg::VisionMarker>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VDT_MSGS__MSG__DETAIL__VISION_MARKER__TRAITS_HPP_
