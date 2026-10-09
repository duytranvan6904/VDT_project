// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from vdt_msgs:msg/PlannerOutput.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "vdt_msgs/msg/detail/planner_output__functions.h"
#include "vdt_msgs/msg/detail/planner_output__struct.hpp"
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

void PlannerOutput_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) vdt_msgs::msg::PlannerOutput(_init);
}

void PlannerOutput_fini_function(void * message_memory)
{
  auto typed_message = static_cast<vdt_msgs::msg::PlannerOutput *>(message_memory);
  typed_message->~PlannerOutput();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PlannerOutput_message_member_array[4] = {
  {
    "vx",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs::msg::PlannerOutput, vx),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vy",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs::msg::PlannerOutput, vy),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vz",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs::msg::PlannerOutput, vz),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "yaw",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs::msg::PlannerOutput, yaw),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PlannerOutput_message_members = {
  "vdt_msgs::msg",  // message namespace
  "PlannerOutput",  // message name
  4,  // number of fields
  sizeof(vdt_msgs::msg::PlannerOutput),
  false,  // has_any_key_member_
  PlannerOutput_message_member_array,  // message members
  PlannerOutput_init_function,  // function to initialize message memory (memory has to be allocated)
  PlannerOutput_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PlannerOutput_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PlannerOutput_message_members,
  get_message_typesupport_handle_function,
  &vdt_msgs__msg__PlannerOutput__get_type_hash,
  &vdt_msgs__msg__PlannerOutput__get_type_description,
  &vdt_msgs__msg__PlannerOutput__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace vdt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<vdt_msgs::msg::PlannerOutput>()
{
  return &::vdt_msgs::msg::rosidl_typesupport_introspection_cpp::PlannerOutput_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vdt_msgs, msg, PlannerOutput)() {
  return &::vdt_msgs::msg::rosidl_typesupport_introspection_cpp::PlannerOutput_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
