// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from vdt_msgs:msg/RcFsmInput.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "vdt_msgs/msg/detail/rc_fsm_input__functions.h"
#include "vdt_msgs/msg/detail/rc_fsm_input__struct.hpp"
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

void RcFsmInput_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) vdt_msgs::msg::RcFsmInput(_init);
}

void RcFsmInput_fini_function(void * message_memory)
{
  auto typed_message = static_cast<vdt_msgs::msg::RcFsmInput *>(message_memory);
  typed_message->~RcFsmInput();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RcFsmInput_message_member_array[2] = {
  {
    "land_switch",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs::msg::RcFsmInput, land_switch),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "kill_switch",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs::msg::RcFsmInput, kill_switch),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RcFsmInput_message_members = {
  "vdt_msgs::msg",  // message namespace
  "RcFsmInput",  // message name
  2,  // number of fields
  sizeof(vdt_msgs::msg::RcFsmInput),
  false,  // has_any_key_member_
  RcFsmInput_message_member_array,  // message members
  RcFsmInput_init_function,  // function to initialize message memory (memory has to be allocated)
  RcFsmInput_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RcFsmInput_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RcFsmInput_message_members,
  get_message_typesupport_handle_function,
  &vdt_msgs__msg__RcFsmInput__get_type_hash,
  &vdt_msgs__msg__RcFsmInput__get_type_description,
  &vdt_msgs__msg__RcFsmInput__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace vdt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<vdt_msgs::msg::RcFsmInput>()
{
  return &::vdt_msgs::msg::rosidl_typesupport_introspection_cpp::RcFsmInput_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vdt_msgs, msg, RcFsmInput)() {
  return &::vdt_msgs::msg::rosidl_typesupport_introspection_cpp::RcFsmInput_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
