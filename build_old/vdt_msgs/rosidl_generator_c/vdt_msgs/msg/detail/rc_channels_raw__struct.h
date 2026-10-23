// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vdt_msgs:msg/RcChannelsRaw.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/rc_channels_raw.h"


#ifndef VDT_MSGS__MSG__DETAIL__RC_CHANNELS_RAW__STRUCT_H_
#define VDT_MSGS__MSG__DETAIL__RC_CHANNELS_RAW__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/RcChannelsRaw in the package vdt_msgs.
typedef struct vdt_msgs__msg__RcChannelsRaw
{
  int16_t ch[16];
  bool valid;
  bool failsafe;
} vdt_msgs__msg__RcChannelsRaw;

// Struct for a sequence of vdt_msgs__msg__RcChannelsRaw.
typedef struct vdt_msgs__msg__RcChannelsRaw__Sequence
{
  vdt_msgs__msg__RcChannelsRaw * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vdt_msgs__msg__RcChannelsRaw__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VDT_MSGS__MSG__DETAIL__RC_CHANNELS_RAW__STRUCT_H_
