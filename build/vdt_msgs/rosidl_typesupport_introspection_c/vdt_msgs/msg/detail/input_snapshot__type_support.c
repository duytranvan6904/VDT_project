// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vdt_msgs:msg/InputSnapshot.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vdt_msgs/msg/detail/input_snapshot__rosidl_typesupport_introspection_c.h"
#include "vdt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vdt_msgs/msg/detail/input_snapshot__functions.h"
#include "vdt_msgs/msg/detail/input_snapshot__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void vdt_msgs__msg__InputSnapshot__rosidl_typesupport_introspection_c__InputSnapshot_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vdt_msgs__msg__InputSnapshot__init(message_memory);
}

void vdt_msgs__msg__InputSnapshot__rosidl_typesupport_introspection_c__InputSnapshot_fini_function(void * message_memory)
{
  vdt_msgs__msg__InputSnapshot__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember vdt_msgs__msg__InputSnapshot__rosidl_typesupport_introspection_c__InputSnapshot_message_member_array[9] = {
  {
    "valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs__msg__InputSnapshot, valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "marker_detected",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs__msg__InputSnapshot, marker_detected),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "align_error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs__msg__InputSnapshot, align_error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "altitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs__msg__InputSnapshot, altitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "delta_h",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs__msg__InputSnapshot, delta_h),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "d_horiz",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs__msg__InputSnapshot, d_horiz),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "touchdown",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs__msg__InputSnapshot, touchdown),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "planner_timeout",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs__msg__InputSnapshot, planner_timeout),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw_rate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs__msg__InputSnapshot, yaw_rate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vdt_msgs__msg__InputSnapshot__rosidl_typesupport_introspection_c__InputSnapshot_message_members = {
  "vdt_msgs__msg",  // message namespace
  "InputSnapshot",  // message name
  9,  // number of fields
  sizeof(vdt_msgs__msg__InputSnapshot),
  false,  // has_any_key_member_
  vdt_msgs__msg__InputSnapshot__rosidl_typesupport_introspection_c__InputSnapshot_message_member_array,  // message members
  vdt_msgs__msg__InputSnapshot__rosidl_typesupport_introspection_c__InputSnapshot_init_function,  // function to initialize message memory (memory has to be allocated)
  vdt_msgs__msg__InputSnapshot__rosidl_typesupport_introspection_c__InputSnapshot_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vdt_msgs__msg__InputSnapshot__rosidl_typesupport_introspection_c__InputSnapshot_message_type_support_handle = {
  0,
  &vdt_msgs__msg__InputSnapshot__rosidl_typesupport_introspection_c__InputSnapshot_message_members,
  get_message_typesupport_handle_function,
  &vdt_msgs__msg__InputSnapshot__get_type_hash,
  &vdt_msgs__msg__InputSnapshot__get_type_description,
  &vdt_msgs__msg__InputSnapshot__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vdt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vdt_msgs, msg, InputSnapshot)() {
  if (!vdt_msgs__msg__InputSnapshot__rosidl_typesupport_introspection_c__InputSnapshot_message_type_support_handle.typesupport_identifier) {
    vdt_msgs__msg__InputSnapshot__rosidl_typesupport_introspection_c__InputSnapshot_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vdt_msgs__msg__InputSnapshot__rosidl_typesupport_introspection_c__InputSnapshot_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
