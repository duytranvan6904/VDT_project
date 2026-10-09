// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vdt_msgs:msg/RcFsmInput.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vdt_msgs/msg/detail/rc_fsm_input__rosidl_typesupport_introspection_c.h"
#include "vdt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vdt_msgs/msg/detail/rc_fsm_input__functions.h"
#include "vdt_msgs/msg/detail/rc_fsm_input__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void vdt_msgs__msg__RcFsmInput__rosidl_typesupport_introspection_c__RcFsmInput_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vdt_msgs__msg__RcFsmInput__init(message_memory);
}

void vdt_msgs__msg__RcFsmInput__rosidl_typesupport_introspection_c__RcFsmInput_fini_function(void * message_memory)
{
  vdt_msgs__msg__RcFsmInput__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember vdt_msgs__msg__RcFsmInput__rosidl_typesupport_introspection_c__RcFsmInput_message_member_array[2] = {
  {
    "land_switch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs__msg__RcFsmInput, land_switch),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "kill_switch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs__msg__RcFsmInput, kill_switch),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vdt_msgs__msg__RcFsmInput__rosidl_typesupport_introspection_c__RcFsmInput_message_members = {
  "vdt_msgs__msg",  // message namespace
  "RcFsmInput",  // message name
  2,  // number of fields
  sizeof(vdt_msgs__msg__RcFsmInput),
  false,  // has_any_key_member_
  vdt_msgs__msg__RcFsmInput__rosidl_typesupport_introspection_c__RcFsmInput_message_member_array,  // message members
  vdt_msgs__msg__RcFsmInput__rosidl_typesupport_introspection_c__RcFsmInput_init_function,  // function to initialize message memory (memory has to be allocated)
  vdt_msgs__msg__RcFsmInput__rosidl_typesupport_introspection_c__RcFsmInput_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vdt_msgs__msg__RcFsmInput__rosidl_typesupport_introspection_c__RcFsmInput_message_type_support_handle = {
  0,
  &vdt_msgs__msg__RcFsmInput__rosidl_typesupport_introspection_c__RcFsmInput_message_members,
  get_message_typesupport_handle_function,
  &vdt_msgs__msg__RcFsmInput__get_type_hash,
  &vdt_msgs__msg__RcFsmInput__get_type_description,
  &vdt_msgs__msg__RcFsmInput__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vdt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vdt_msgs, msg, RcFsmInput)() {
  if (!vdt_msgs__msg__RcFsmInput__rosidl_typesupport_introspection_c__RcFsmInput_message_type_support_handle.typesupport_identifier) {
    vdt_msgs__msg__RcFsmInput__rosidl_typesupport_introspection_c__RcFsmInput_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vdt_msgs__msg__RcFsmInput__rosidl_typesupport_introspection_c__RcFsmInput_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
