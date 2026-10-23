// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from vdt_msgs:msg/InputSnapshot.idl
// generated code does not contain a copyright notice

#ifndef VDT_MSGS__MSG__DETAIL__INPUT_SNAPSHOT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define VDT_MSGS__MSG__DETAIL__INPUT_SNAPSHOT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "vdt_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "vdt_msgs/msg/detail/input_snapshot__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace vdt_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
cdr_serialize(
  const vdt_msgs::msg::InputSnapshot & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  vdt_msgs::msg::InputSnapshot & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
get_serialized_size(
  const vdt_msgs::msg::InputSnapshot & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
max_serialized_size_InputSnapshot(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
cdr_serialize_key(
  const vdt_msgs::msg::InputSnapshot & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
get_serialized_size_key(
  const vdt_msgs::msg::InputSnapshot & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
max_serialized_size_key_InputSnapshot(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace vdt_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vdt_msgs, msg, InputSnapshot)();

#ifdef __cplusplus
}
#endif

#endif  // VDT_MSGS__MSG__DETAIL__INPUT_SNAPSHOT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
