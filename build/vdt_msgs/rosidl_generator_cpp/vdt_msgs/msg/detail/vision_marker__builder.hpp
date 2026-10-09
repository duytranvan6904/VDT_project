// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vdt_msgs:msg/VisionMarker.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/vision_marker.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__VISION_MARKER__BUILDER_HPP_
#define VDT_MSGS__MSG__DETAIL__VISION_MARKER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vdt_msgs/msg/detail/vision_marker__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vdt_msgs
{

namespace msg
{

namespace builder
{

class Init_VisionMarker_pixel_align_error
{
public:
  explicit Init_VisionMarker_pixel_align_error(::vdt_msgs::msg::VisionMarker & msg)
  : msg_(msg)
  {}
  ::vdt_msgs::msg::VisionMarker pixel_align_error(::vdt_msgs::msg::VisionMarker::_pixel_align_error_type arg)
  {
    msg_.pixel_align_error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vdt_msgs::msg::VisionMarker msg_;
};

class Init_VisionMarker_marker_visible
{
public:
  Init_VisionMarker_marker_visible()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VisionMarker_pixel_align_error marker_visible(::vdt_msgs::msg::VisionMarker::_marker_visible_type arg)
  {
    msg_.marker_visible = std::move(arg);
    return Init_VisionMarker_pixel_align_error(msg_);
  }

private:
  ::vdt_msgs::msg::VisionMarker msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vdt_msgs::msg::VisionMarker>()
{
  return vdt_msgs::msg::builder::Init_VisionMarker_marker_visible();
}

}  // namespace vdt_msgs

#endif  // VDT_MSGS__MSG__DETAIL__VISION_MARKER__BUILDER_HPP_
