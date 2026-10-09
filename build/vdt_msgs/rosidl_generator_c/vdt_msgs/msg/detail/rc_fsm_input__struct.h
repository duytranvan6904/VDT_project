// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vdt_msgs:msg/RcFsmInput.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/rc_fsm_input.h"


#ifndef VDT_MSGS__MSG__DETAIL__RC_FSM_INPUT__STRUCT_H_
#define VDT_MSGS__MSG__DETAIL__RC_FSM_INPUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/RcFsmInput in the package vdt_msgs.
typedef struct vdt_msgs__msg__RcFsmInput
{
  bool land_switch;
  bool kill_switch;
} vdt_msgs__msg__RcFsmInput;

// Struct for a sequence of vdt_msgs__msg__RcFsmInput.
typedef struct vdt_msgs__msg__RcFsmInput__Sequence
{
  vdt_msgs__msg__RcFsmInput * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vdt_msgs__msg__RcFsmInput__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VDT_MSGS__MSG__DETAIL__RC_FSM_INPUT__STRUCT_H_
