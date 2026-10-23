// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vdt_msgs:msg/AltEstimate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/alt_estimate.h"


#ifndef VDT_MSGS__MSG__DETAIL__ALT_ESTIMATE__STRUCT_H_
#define VDT_MSGS__MSG__DETAIL__ALT_ESTIMATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/AltEstimate in the package vdt_msgs.
typedef struct vdt_msgs__msg__AltEstimate
{
  float altitude;
  bool touchdown_flag;
} vdt_msgs__msg__AltEstimate;

// Struct for a sequence of vdt_msgs__msg__AltEstimate.
typedef struct vdt_msgs__msg__AltEstimate__Sequence
{
  vdt_msgs__msg__AltEstimate * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vdt_msgs__msg__AltEstimate__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VDT_MSGS__MSG__DETAIL__ALT_ESTIMATE__STRUCT_H_
