// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vdt_msgs:msg/PlannerOutput.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/planner_output.h"


#ifndef VDT_MSGS__MSG__DETAIL__PLANNER_OUTPUT__STRUCT_H_
#define VDT_MSGS__MSG__DETAIL__PLANNER_OUTPUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/PlannerOutput in the package vdt_msgs.
typedef struct vdt_msgs__msg__PlannerOutput
{
  float vx;
  float vy;
  float vz;
  float yaw;
} vdt_msgs__msg__PlannerOutput;

// Struct for a sequence of vdt_msgs__msg__PlannerOutput.
typedef struct vdt_msgs__msg__PlannerOutput__Sequence
{
  vdt_msgs__msg__PlannerOutput * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vdt_msgs__msg__PlannerOutput__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VDT_MSGS__MSG__DETAIL__PLANNER_OUTPUT__STRUCT_H_
