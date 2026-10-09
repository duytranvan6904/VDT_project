// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vdt_msgs:msg/RcChannelsRaw.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vdt_msgs/msg/detail/rc_channels_raw__rosidl_typesupport_introspection_c.h"
#include "vdt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vdt_msgs/msg/detail/rc_channels_raw__functions.h"
#include "vdt_msgs/msg/detail/rc_channels_raw__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__RcChannelsRaw_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vdt_msgs__msg__RcChannelsRaw__init(message_memory);
}

void vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__RcChannelsRaw_fini_function(void * message_memory)
{
  vdt_msgs__msg__RcChannelsRaw__fini(message_memory);
}

size_t vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__size_function__RcChannelsRaw__ch(
  const void * untyped_member)
{
  (void)untyped_member;
  return 16;
}

const void * vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__get_const_function__RcChannelsRaw__ch(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__get_function__RcChannelsRaw__ch(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__fetch_function__RcChannelsRaw__ch(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__get_const_function__RcChannelsRaw__ch(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__assign_function__RcChannelsRaw__ch(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__get_function__RcChannelsRaw__ch(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__RcChannelsRaw_message_member_array[3] = {
  {
    "ch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    16,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs__msg__RcChannelsRaw, ch),  // bytes offset in struct
    NULL,  // default value
    vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__size_function__RcChannelsRaw__ch,  // size() function pointer
    vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__get_const_function__RcChannelsRaw__ch,  // get_const(index) function pointer
    vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__get_function__RcChannelsRaw__ch,  // get(index) function pointer
    vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__fetch_function__RcChannelsRaw__ch,  // fetch(index, &value) function pointer
    vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__assign_function__RcChannelsRaw__ch,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs__msg__RcChannelsRaw, valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "failsafe",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs__msg__RcChannelsRaw, failsafe),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__RcChannelsRaw_message_members = {
  "vdt_msgs__msg",  // message namespace
  "RcChannelsRaw",  // message name
  3,  // number of fields
  sizeof(vdt_msgs__msg__RcChannelsRaw),
  false,  // has_any_key_member_
  vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__RcChannelsRaw_message_member_array,  // message members
  vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__RcChannelsRaw_init_function,  // function to initialize message memory (memory has to be allocated)
  vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__RcChannelsRaw_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__RcChannelsRaw_message_type_support_handle = {
  0,
  &vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__RcChannelsRaw_message_members,
  get_message_typesupport_handle_function,
  &vdt_msgs__msg__RcChannelsRaw__get_type_hash,
  &vdt_msgs__msg__RcChannelsRaw__get_type_description,
  &vdt_msgs__msg__RcChannelsRaw__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vdt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vdt_msgs, msg, RcChannelsRaw)() {
  if (!vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__RcChannelsRaw_message_type_support_handle.typesupport_identifier) {
    vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__RcChannelsRaw_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vdt_msgs__msg__RcChannelsRaw__rosidl_typesupport_introspection_c__RcChannelsRaw_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
