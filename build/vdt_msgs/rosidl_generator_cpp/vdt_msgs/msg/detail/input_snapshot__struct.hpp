// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vdt_msgs:msg/InputSnapshot.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/input_snapshot.hpp"


#ifndef VDT_MSGS__MSG__DETAIL__INPUT_SNAPSHOT__STRUCT_HPP_
#define VDT_MSGS__MSG__DETAIL__INPUT_SNAPSHOT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vdt_msgs__msg__InputSnapshot __attribute__((deprecated))
#else
# define DEPRECATED__vdt_msgs__msg__InputSnapshot __declspec(deprecated)
#endif

namespace vdt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct InputSnapshot_
{
  using Type = InputSnapshot_<ContainerAllocator>;

  explicit InputSnapshot_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->valid = false;
      this->marker_detected = false;
      this->align_error = 0.0f;
      this->altitude = 0.0f;
      this->delta_h = 0.0f;
      this->d_horiz = 0.0f;
      this->touchdown = false;
      this->planner_timeout = false;
      this->yaw_rate = 0.0f;
    }
  }

  explicit InputSnapshot_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->valid = false;
      this->marker_detected = false;
      this->align_error = 0.0f;
      this->altitude = 0.0f;
      this->delta_h = 0.0f;
      this->d_horiz = 0.0f;
      this->touchdown = false;
      this->planner_timeout = false;
      this->yaw_rate = 0.0f;
    }
  }

  // field types and members
  using _valid_type =
    bool;
  _valid_type valid;
  using _marker_detected_type =
    bool;
  _marker_detected_type marker_detected;
  using _align_error_type =
    float;
  _align_error_type align_error;
  using _altitude_type =
    float;
  _altitude_type altitude;
  using _delta_h_type =
    float;
  _delta_h_type delta_h;
  using _d_horiz_type =
    float;
  _d_horiz_type d_horiz;
  using _touchdown_type =
    bool;
  _touchdown_type touchdown;
  using _planner_timeout_type =
    bool;
  _planner_timeout_type planner_timeout;
  using _yaw_rate_type =
    float;
  _yaw_rate_type yaw_rate;

  // setters for named parameter idiom
  Type & set__valid(
    const bool & _arg)
  {
    this->valid = _arg;
    return *this;
  }
  Type & set__marker_detected(
    const bool & _arg)
  {
    this->marker_detected = _arg;
    return *this;
  }
  Type & set__align_error(
    const float & _arg)
  {
    this->align_error = _arg;
    return *this;
  }
  Type & set__altitude(
    const float & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__delta_h(
    const float & _arg)
  {
    this->delta_h = _arg;
    return *this;
  }
  Type & set__d_horiz(
    const float & _arg)
  {
    this->d_horiz = _arg;
    return *this;
  }
  Type & set__touchdown(
    const bool & _arg)
  {
    this->touchdown = _arg;
    return *this;
  }
  Type & set__planner_timeout(
    const bool & _arg)
  {
    this->planner_timeout = _arg;
    return *this;
  }
  Type & set__yaw_rate(
    const float & _arg)
  {
    this->yaw_rate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vdt_msgs::msg::InputSnapshot_<ContainerAllocator> *;
  using ConstRawPtr =
    const vdt_msgs::msg::InputSnapshot_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vdt_msgs::msg::InputSnapshot_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vdt_msgs::msg::InputSnapshot_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vdt_msgs::msg::InputSnapshot_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vdt_msgs::msg::InputSnapshot_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vdt_msgs::msg::InputSnapshot_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vdt_msgs::msg::InputSnapshot_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vdt_msgs::msg::InputSnapshot_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vdt_msgs::msg::InputSnapshot_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vdt_msgs__msg__InputSnapshot
    std::shared_ptr<vdt_msgs::msg::InputSnapshot_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vdt_msgs__msg__InputSnapshot
    std::shared_ptr<vdt_msgs::msg::InputSnapshot_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InputSnapshot_ & other) const
  {
    if (this->valid != other.valid) {
      return false;
    }
    if (this->marker_detected != other.marker_detected) {
      return false;
    }
    if (this->align_error != other.align_error) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->delta_h != other.delta_h) {
      return false;
    }
    if (this->d_horiz != other.d_horiz) {
      return false;
    }
    if (this->touchdown != other.touchdown) {
      return false;
    }
    if (this->planner_timeout != other.planner_timeout) {
      return false;
    }
    if (this->yaw_rate != other.yaw_rate) {
      return false;
    }
    return true;
  }
  bool operator!=(const InputSnapshot_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InputSnapshot_

// alias to use template instance with default allocator
using InputSnapshot =
  vdt_msgs::msg::InputSnapshot_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vdt_msgs

#endif  // VDT_MSGS__MSG__DETAIL__INPUT_SNAPSHOT__STRUCT_HPP_
