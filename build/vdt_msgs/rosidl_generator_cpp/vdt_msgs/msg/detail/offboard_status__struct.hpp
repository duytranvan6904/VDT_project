// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vdt_msgs:msg/OffboardStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/offboard_status.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__OFFBOARD_STATUS__STRUCT_HPP_
#define VDT_MSGS__MSG__DETAIL__OFFBOARD_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vdt_msgs__msg__OffboardStatus __attribute__((deprecated))
#else
# define DEPRECATED__vdt_msgs__msg__OffboardStatus __declspec(deprecated)
#endif

namespace vdt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OffboardStatus_
{
  using Type = OffboardStatus_<ContainerAllocator>;

  explicit OffboardStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->offboard_active = false;
      this->heartbeat_age_sec = 0.0f;
    }
  }

  explicit OffboardStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->offboard_active = false;
      this->heartbeat_age_sec = 0.0f;
    }
  }

  // field types and members
  using _offboard_active_type =
    bool;
  _offboard_active_type offboard_active;
  using _heartbeat_age_sec_type =
    float;
  _heartbeat_age_sec_type heartbeat_age_sec;

  // setters for named parameter idiom
  Type & set__offboard_active(
    const bool & _arg)
  {
    this->offboard_active = _arg;
    return *this;
  }
  Type & set__heartbeat_age_sec(
    const float & _arg)
  {
    this->heartbeat_age_sec = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vdt_msgs::msg::OffboardStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const vdt_msgs::msg::OffboardStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vdt_msgs::msg::OffboardStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vdt_msgs::msg::OffboardStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vdt_msgs::msg::OffboardStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vdt_msgs::msg::OffboardStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vdt_msgs::msg::OffboardStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vdt_msgs::msg::OffboardStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vdt_msgs::msg::OffboardStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vdt_msgs::msg::OffboardStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vdt_msgs__msg__OffboardStatus
    std::shared_ptr<vdt_msgs::msg::OffboardStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vdt_msgs__msg__OffboardStatus
    std::shared_ptr<vdt_msgs::msg::OffboardStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OffboardStatus_ & other) const
  {
    if (this->offboard_active != other.offboard_active) {
      return false;
    }
    if (this->heartbeat_age_sec != other.heartbeat_age_sec) {
      return false;
    }
    return true;
  }
  bool operator!=(const OffboardStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OffboardStatus_

// alias to use template instance with default allocator
using OffboardStatus =
  vdt_msgs::msg::OffboardStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vdt_msgs

#endif  // VDT_MSGS__MSG__DETAIL__OFFBOARD_STATUS__STRUCT_HPP_
