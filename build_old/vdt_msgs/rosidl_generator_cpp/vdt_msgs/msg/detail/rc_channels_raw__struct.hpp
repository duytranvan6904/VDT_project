// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vdt_msgs:msg/RcChannelsRaw.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/rc_channels_raw.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__RC_CHANNELS_RAW__STRUCT_HPP_
#define VDT_MSGS__MSG__DETAIL__RC_CHANNELS_RAW__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vdt_msgs__msg__RcChannelsRaw __attribute__((deprecated))
#else
# define DEPRECATED__vdt_msgs__msg__RcChannelsRaw __declspec(deprecated)
#endif

namespace vdt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RcChannelsRaw_
{
  using Type = RcChannelsRaw_<ContainerAllocator>;

  explicit RcChannelsRaw_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<int16_t, 16>::iterator, int16_t>(this->ch.begin(), this->ch.end(), 0);
      this->valid = false;
      this->failsafe = false;
    }
  }

  explicit RcChannelsRaw_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : ch(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<int16_t, 16>::iterator, int16_t>(this->ch.begin(), this->ch.end(), 0);
      this->valid = false;
      this->failsafe = false;
    }
  }

  // field types and members
  using _ch_type =
    std::array<int16_t, 16>;
  _ch_type ch;
  using _valid_type =
    bool;
  _valid_type valid;
  using _failsafe_type =
    bool;
  _failsafe_type failsafe;

  // setters for named parameter idiom
  Type & set__ch(
    const std::array<int16_t, 16> & _arg)
  {
    this->ch = _arg;
    return *this;
  }
  Type & set__valid(
    const bool & _arg)
  {
    this->valid = _arg;
    return *this;
  }
  Type & set__failsafe(
    const bool & _arg)
  {
    this->failsafe = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vdt_msgs::msg::RcChannelsRaw_<ContainerAllocator> *;
  using ConstRawPtr =
    const vdt_msgs::msg::RcChannelsRaw_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vdt_msgs::msg::RcChannelsRaw_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vdt_msgs::msg::RcChannelsRaw_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vdt_msgs::msg::RcChannelsRaw_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vdt_msgs::msg::RcChannelsRaw_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vdt_msgs::msg::RcChannelsRaw_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vdt_msgs::msg::RcChannelsRaw_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vdt_msgs::msg::RcChannelsRaw_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vdt_msgs::msg::RcChannelsRaw_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vdt_msgs__msg__RcChannelsRaw
    std::shared_ptr<vdt_msgs::msg::RcChannelsRaw_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vdt_msgs__msg__RcChannelsRaw
    std::shared_ptr<vdt_msgs::msg::RcChannelsRaw_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RcChannelsRaw_ & other) const
  {
    if (this->ch != other.ch) {
      return false;
    }
    if (this->valid != other.valid) {
      return false;
    }
    if (this->failsafe != other.failsafe) {
      return false;
    }
    return true;
  }
  bool operator!=(const RcChannelsRaw_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RcChannelsRaw_

// alias to use template instance with default allocator
using RcChannelsRaw =
  vdt_msgs::msg::RcChannelsRaw_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vdt_msgs

#endif  // VDT_MSGS__MSG__DETAIL__RC_CHANNELS_RAW__STRUCT_HPP_
