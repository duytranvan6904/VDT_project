// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vdt_msgs:msg/TimeoutFlags.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/timeout_flags.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__TIMEOUT_FLAGS__STRUCT_HPP_
#define VDT_MSGS__MSG__DETAIL__TIMEOUT_FLAGS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vdt_msgs__msg__TimeoutFlags __attribute__((deprecated))
#else
# define DEPRECATED__vdt_msgs__msg__TimeoutFlags __declspec(deprecated)
#endif

namespace vdt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TimeoutFlags_
{
  using Type = TimeoutFlags_<ContainerAllocator>;

  explicit TimeoutFlags_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ekf_timeout = false;
      this->vision_timeout = false;
      this->alt_timeout = false;
      this->planner_timeout = false;
    }
  }

  explicit TimeoutFlags_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ekf_timeout = false;
      this->vision_timeout = false;
      this->alt_timeout = false;
      this->planner_timeout = false;
    }
  }

  // field types and members
  using _ekf_timeout_type =
    bool;
  _ekf_timeout_type ekf_timeout;
  using _vision_timeout_type =
    bool;
  _vision_timeout_type vision_timeout;
  using _alt_timeout_type =
    bool;
  _alt_timeout_type alt_timeout;
  using _planner_timeout_type =
    bool;
  _planner_timeout_type planner_timeout;

  // setters for named parameter idiom
  Type & set__ekf_timeout(
    const bool & _arg)
  {
    this->ekf_timeout = _arg;
    return *this;
  }
  Type & set__vision_timeout(
    const bool & _arg)
  {
    this->vision_timeout = _arg;
    return *this;
  }
  Type & set__alt_timeout(
    const bool & _arg)
  {
    this->alt_timeout = _arg;
    return *this;
  }
  Type & set__planner_timeout(
    const bool & _arg)
  {
    this->planner_timeout = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vdt_msgs::msg::TimeoutFlags_<ContainerAllocator> *;
  using ConstRawPtr =
    const vdt_msgs::msg::TimeoutFlags_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vdt_msgs::msg::TimeoutFlags_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vdt_msgs::msg::TimeoutFlags_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vdt_msgs::msg::TimeoutFlags_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vdt_msgs::msg::TimeoutFlags_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vdt_msgs::msg::TimeoutFlags_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vdt_msgs::msg::TimeoutFlags_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vdt_msgs::msg::TimeoutFlags_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vdt_msgs::msg::TimeoutFlags_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vdt_msgs__msg__TimeoutFlags
    std::shared_ptr<vdt_msgs::msg::TimeoutFlags_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vdt_msgs__msg__TimeoutFlags
    std::shared_ptr<vdt_msgs::msg::TimeoutFlags_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TimeoutFlags_ & other) const
  {
    if (this->ekf_timeout != other.ekf_timeout) {
      return false;
    }
    if (this->vision_timeout != other.vision_timeout) {
      return false;
    }
    if (this->alt_timeout != other.alt_timeout) {
      return false;
    }
    if (this->planner_timeout != other.planner_timeout) {
      return false;
    }
    return true;
  }
  bool operator!=(const TimeoutFlags_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TimeoutFlags_

// alias to use template instance with default allocator
using TimeoutFlags =
  vdt_msgs::msg::TimeoutFlags_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vdt_msgs

#endif  // VDT_MSGS__MSG__DETAIL__TIMEOUT_FLAGS__STRUCT_HPP_
