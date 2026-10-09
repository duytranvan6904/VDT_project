// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vdt_msgs:msg/VisionMarker.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/vision_marker.h"


#ifndef VDT_MSGS__MSG__DETAIL__VISION_MARKER__STRUCT_H_
#define VDT_MSGS__MSG__DETAIL__VISION_MARKER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/VisionMarker in the package vdt_msgs.
typedef struct vdt_msgs__msg__VisionMarker
{
  bool marker_visible;
  float pixel_align_error;
} vdt_msgs__msg__VisionMarker;

// Struct for a sequence of vdt_msgs__msg__VisionMarker.
typedef struct vdt_msgs__msg__VisionMarker__Sequence
{
  vdt_msgs__msg__VisionMarker * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vdt_msgs__msg__VisionMarker__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VDT_MSGS__MSG__DETAIL__VISION_MARKER__STRUCT_H_
