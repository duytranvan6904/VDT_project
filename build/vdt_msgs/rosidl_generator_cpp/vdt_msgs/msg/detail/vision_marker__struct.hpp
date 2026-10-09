// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vdt_msgs:msg/VisionMarker.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/vision_marker.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__VISION_MARKER__STRUCT_HPP_
#define VDT_MSGS__MSG__DETAIL__VISION_MARKER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vdt_msgs__msg__VisionMarker __attribute__((deprecated))
#else
# define DEPRECATED__vdt_msgs__msg__VisionMarker __declspec(deprecated)
#endif

namespace vdt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VisionMarker_
{
  using Type = VisionMarker_<ContainerAllocator>;

  explicit VisionMarker_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->marker_visible = false;
      this->pixel_align_error = 0.0f;
    }
  }

  explicit VisionMarker_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->marker_visible = false;
      this->pixel_align_error = 0.0f;
    }
  }

  // field types and members
  using _marker_visible_type =
    bool;
  _marker_visible_type marker_visible;
  using _pixel_align_error_type =
    float;
  _pixel_align_error_type pixel_align_error;

  // setters for named parameter idiom
  Type & set__marker_visible(
    const bool & _arg)
  {
    this->marker_visible = _arg;
    return *this;
  }
  Type & set__pixel_align_error(
    const float & _arg)
  {
    this->pixel_align_error = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vdt_msgs::msg::VisionMarker_<ContainerAllocator> *;
  using ConstRawPtr =
    const vdt_msgs::msg::VisionMarker_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vdt_msgs::msg::VisionMarker_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vdt_msgs::msg::VisionMarker_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vdt_msgs::msg::VisionMarker_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vdt_msgs::msg::VisionMarker_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vdt_msgs::msg::VisionMarker_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vdt_msgs::msg::VisionMarker_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vdt_msgs::msg::VisionMarker_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vdt_msgs::msg::VisionMarker_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vdt_msgs__msg__VisionMarker
    std::shared_ptr<vdt_msgs::msg::VisionMarker_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vdt_msgs__msg__VisionMarker
    std::shared_ptr<vdt_msgs::msg::VisionMarker_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VisionMarker_ & other) const
  {
    if (this->marker_visible != other.marker_visible) {
      return false;
    }
    if (this->pixel_align_error != other.pixel_align_error) {
      return false;
    }
    return true;
  }
  bool operator!=(const VisionMarker_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VisionMarker_

// alias to use template instance with default allocator
using VisionMarker =
  vdt_msgs::msg::VisionMarker_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vdt_msgs

#endif  // VDT_MSGS__MSG__DETAIL__VISION_MARKER__STRUCT_HPP_
