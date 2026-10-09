// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from vdt_msgs:msg/TimeoutFlags.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "vdt_msgs/msg/detail/timeout_flags__functions.h"
#include "vdt_msgs/msg/detail/timeout_flags__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace vdt_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TimeoutFlags_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) vdt_msgs::msg::TimeoutFlags(_init);
}

void TimeoutFlags_fini_function(void * message_memory)
{
  auto typed_message = static_cast<vdt_msgs::msg::TimeoutFlags *>(message_memory);
  typed_message->~TimeoutFlags();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TimeoutFlags_message_member_array[4] = {
  {
    "ekf_timeout",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs::msg::TimeoutFlags, ekf_timeout),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vision_timeout",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs::msg::TimeoutFlags, vision_timeout),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "alt_timeout",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs::msg::TimeoutFlags, alt_timeout),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "planner_timeout",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs::msg::TimeoutFlags, planner_timeout),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TimeoutFlags_message_members = {
  "vdt_msgs::msg",  // message namespace
  "TimeoutFlags",  // message name
  4,  // number of fields
  sizeof(vdt_msgs::msg::TimeoutFlags),
  false,  // has_any_key_member_
  TimeoutFlags_message_member_array,  // message members
  TimeoutFlags_init_function,  // function to initialize message memory (memory has to be allocated)
  TimeoutFlags_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TimeoutFlags_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TimeoutFlags_message_members,
  get_message_typesupport_handle_function,
  &vdt_msgs__msg__TimeoutFlags__get_type_hash,
  &vdt_msgs__msg__TimeoutFlags__get_type_description,
  &vdt_msgs__msg__TimeoutFlags__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace vdt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<vdt_msgs::msg::TimeoutFlags>()
{
  return &::vdt_msgs::msg::rosidl_typesupport_introspection_cpp::TimeoutFlags_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vdt_msgs, msg, TimeoutFlags)() {
  return &::vdt_msgs::msg::rosidl_typesupport_introspection_cpp::TimeoutFlags_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
