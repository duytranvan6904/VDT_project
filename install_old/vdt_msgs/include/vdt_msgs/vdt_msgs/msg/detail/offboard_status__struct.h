// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vdt_msgs:msg/OffboardStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/offboard_status.h"


#ifndef VDT_MSGS__MSG__DETAIL__OFFBOARD_STATUS__STRUCT_H_
#define VDT_MSGS__MSG__DETAIL__OFFBOARD_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/OffboardStatus in the package vdt_msgs.
typedef struct vdt_msgs__msg__OffboardStatus
{
  bool offboard_active;
  float heartbeat_age_sec;
} vdt_msgs__msg__OffboardStatus;

// Struct for a sequence of vdt_msgs__msg__OffboardStatus.
typedef struct vdt_msgs__msg__OffboardStatus__Sequence
{
  vdt_msgs__msg__OffboardStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vdt_msgs__msg__OffboardStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VDT_MSGS__MSG__DETAIL__OFFBOARD_STATUS__STRUCT_H_
