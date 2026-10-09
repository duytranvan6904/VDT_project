// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vdt_msgs:msg/InputSnapshot.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/input_snapshot.h"


#ifndef VDT_MSGS__MSG__DETAIL__INPUT_SNAPSHOT__STRUCT_H_
#define VDT_MSGS__MSG__DETAIL__INPUT_SNAPSHOT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/InputSnapshot in the package vdt_msgs.
typedef struct vdt_msgs__msg__InputSnapshot
{
  bool valid;
  bool marker_detected;
  float align_error;
  float altitude;
  float delta_h;
  float d_horiz;
  bool touchdown;
  bool planner_timeout;
  float yaw_rate;
} vdt_msgs__msg__InputSnapshot;

// Struct for a sequence of vdt_msgs__msg__InputSnapshot.
typedef struct vdt_msgs__msg__InputSnapshot__Sequence
{
  vdt_msgs__msg__InputSnapshot * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vdt_msgs__msg__InputSnapshot__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VDT_MSGS__MSG__DETAIL__INPUT_SNAPSHOT__STRUCT_H_
