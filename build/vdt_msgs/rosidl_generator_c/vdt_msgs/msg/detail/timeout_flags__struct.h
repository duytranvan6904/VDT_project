// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vdt_msgs:msg/TimeoutFlags.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/timeout_flags.h"


#ifndef VDT_MSGS__MSG__DETAIL__TIMEOUT_FLAGS__STRUCT_H_
#define VDT_MSGS__MSG__DETAIL__TIMEOUT_FLAGS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/TimeoutFlags in the package vdt_msgs.
typedef struct vdt_msgs__msg__TimeoutFlags
{
  bool ekf_timeout;
  bool vision_timeout;
  bool alt_timeout;
  bool planner_timeout;
} vdt_msgs__msg__TimeoutFlags;

// Struct for a sequence of vdt_msgs__msg__TimeoutFlags.
typedef struct vdt_msgs__msg__TimeoutFlags__Sequence
{
  vdt_msgs__msg__TimeoutFlags * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vdt_msgs__msg__TimeoutFlags__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VDT_MSGS__MSG__DETAIL__TIMEOUT_FLAGS__STRUCT_H_
