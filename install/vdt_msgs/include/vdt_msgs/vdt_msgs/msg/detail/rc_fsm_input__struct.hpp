// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vdt_msgs:msg/RcFsmInput.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/rc_fsm_input.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__RC_FSM_INPUT__STRUCT_HPP_
#define VDT_MSGS__MSG__DETAIL__RC_FSM_INPUT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vdt_msgs__msg__RcFsmInput __attribute__((deprecated))
#else
# define DEPRECATED__vdt_msgs__msg__RcFsmInput __declspec(deprecated)
#endif

namespace vdt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RcFsmInput_
{
  using Type = RcFsmInput_<ContainerAllocator>;

  explicit RcFsmInput_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->land_switch = false;
      this->kill_switch = false;
    }
  }

  explicit RcFsmInput_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->land_switch = false;
      this->kill_switch = false;
    }
  }

  // field types and members
  using _land_switch_type =
    bool;
  _land_switch_type land_switch;
  using _kill_switch_type =
    bool;
  _kill_switch_type kill_switch;

  // setters for named parameter idiom
  Type & set__land_switch(
    const bool & _arg)
  {
    this->land_switch = _arg;
    return *this;
  }
  Type & set__kill_switch(
    const bool & _arg)
  {
    this->kill_switch = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vdt_msgs::msg::RcFsmInput_<ContainerAllocator> *;
  using ConstRawPtr =
    const vdt_msgs::msg::RcFsmInput_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vdt_msgs::msg::RcFsmInput_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vdt_msgs::msg::RcFsmInput_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vdt_msgs::msg::RcFsmInput_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vdt_msgs::msg::RcFsmInput_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vdt_msgs::msg::RcFsmInput_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vdt_msgs::msg::RcFsmInput_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vdt_msgs::msg::RcFsmInput_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vdt_msgs::msg::RcFsmInput_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vdt_msgs__msg__RcFsmInput
    std::shared_ptr<vdt_msgs::msg::RcFsmInput_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vdt_msgs__msg__RcFsmInput
    std::shared_ptr<vdt_msgs::msg::RcFsmInput_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RcFsmInput_ & other) const
  {
    if (this->land_switch != other.land_switch) {
      return false;
    }
    if (this->kill_switch != other.kill_switch) {
      return false;
    }
    return true;
  }
  bool operator!=(const RcFsmInput_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RcFsmInput_

// alias to use template instance with default allocator
using RcFsmInput =
  vdt_msgs::msg::RcFsmInput_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vdt_msgs

#endif  // VDT_MSGS__MSG__DETAIL__RC_FSM_INPUT__STRUCT_HPP_
