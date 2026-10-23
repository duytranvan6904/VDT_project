// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vdt_msgs:msg/AltEstimate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/alt_estimate.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__ALT_ESTIMATE__STRUCT_HPP_
#define VDT_MSGS__MSG__DETAIL__ALT_ESTIMATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vdt_msgs__msg__AltEstimate __attribute__((deprecated))
#else
# define DEPRECATED__vdt_msgs__msg__AltEstimate __declspec(deprecated)
#endif

namespace vdt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AltEstimate_
{
  using Type = AltEstimate_<ContainerAllocator>;

  explicit AltEstimate_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->altitude = 0.0f;
      this->touchdown_flag = false;
    }
  }

  explicit AltEstimate_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->altitude = 0.0f;
      this->touchdown_flag = false;
    }
  }

  // field types and members
  using _altitude_type =
    float;
  _altitude_type altitude;
  using _touchdown_flag_type =
    bool;
  _touchdown_flag_type touchdown_flag;

  // setters for named parameter idiom
  Type & set__altitude(
    const float & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__touchdown_flag(
    const bool & _arg)
  {
    this->touchdown_flag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vdt_msgs::msg::AltEstimate_<ContainerAllocator> *;
  using ConstRawPtr =
    const vdt_msgs::msg::AltEstimate_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vdt_msgs::msg::AltEstimate_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vdt_msgs::msg::AltEstimate_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vdt_msgs::msg::AltEstimate_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vdt_msgs::msg::AltEstimate_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vdt_msgs::msg::AltEstimate_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vdt_msgs::msg::AltEstimate_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vdt_msgs::msg::AltEstimate_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vdt_msgs::msg::AltEstimate_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vdt_msgs__msg__AltEstimate
    std::shared_ptr<vdt_msgs::msg::AltEstimate_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vdt_msgs__msg__AltEstimate
    std::shared_ptr<vdt_msgs::msg::AltEstimate_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AltEstimate_ & other) const
  {
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->touchdown_flag != other.touchdown_flag) {
      return false;
    }
    return true;
  }
  bool operator!=(const AltEstimate_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AltEstimate_

// alias to use template instance with default allocator
using AltEstimate =
  vdt_msgs::msg::AltEstimate_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vdt_msgs

#endif  // VDT_MSGS__MSG__DETAIL__ALT_ESTIMATE__STRUCT_HPP_
