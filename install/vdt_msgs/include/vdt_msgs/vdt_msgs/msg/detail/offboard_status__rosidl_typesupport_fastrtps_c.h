// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from vdt_msgs:msg/OffboardStatus.idl
// generated code does not contain a copyright notice
#ifndef VDT_MSGS__MSG__DETAIL__OFFBOARD_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define VDT_MSGS__MSG__DETAIL__OFFBOARD_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "vdt_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "vdt_msgs/msg/detail/offboard_status__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
bool cdr_serialize_vdt_msgs__msg__OffboardStatus(
  const vdt_msgs__msg__OffboardStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
bool cdr_deserialize_vdt_msgs__msg__OffboardStatus(
  eprosima::fastcdr::Cdr &,
  vdt_msgs__msg__OffboardStatus * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
size_t get_serialized_size_vdt_msgs__msg__OffboardStatus(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
size_t max_serialized_size_vdt_msgs__msg__OffboardStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
bool cdr_serialize_key_vdt_msgs__msg__OffboardStatus(
  const vdt_msgs__msg__OffboardStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
size_t get_serialized_size_key_vdt_msgs__msg__OffboardStatus(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
size_t max_serialized_size_key_vdt_msgs__msg__OffboardStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vdt_msgs, msg, OffboardStatus)();

#ifdef __cplusplus
}
#endif

#endif  // VDT_MSGS__MSG__DETAIL__OFFBOARD_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
