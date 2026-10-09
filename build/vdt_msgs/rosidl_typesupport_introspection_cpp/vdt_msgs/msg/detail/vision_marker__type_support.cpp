// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from vdt_msgs:msg/VisionMarker.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "vdt_msgs/msg/detail/vision_marker__functions.h"
#include "vdt_msgs/msg/detail/vision_marker__struct.hpp"
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

void VisionMarker_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) vdt_msgs::msg::VisionMarker(_init);
}

void VisionMarker_fini_function(void * message_memory)
{
  auto typed_message = static_cast<vdt_msgs::msg::VisionMarker *>(message_memory);
  typed_message->~VisionMarker();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember VisionMarker_message_member_array[2] = {
  {
    "marker_visible",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs::msg::VisionMarker, marker_visible),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pixel_align_error",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs::msg::VisionMarker, pixel_align_error),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers VisionMarker_message_members = {
  "vdt_msgs::msg",  // message namespace
  "VisionMarker",  // message name
  2,  // number of fields
  sizeof(vdt_msgs::msg::VisionMarker),
  false,  // has_any_key_member_
  VisionMarker_message_member_array,  // message members
  VisionMarker_init_function,  // function to initialize message memory (memory has to be allocated)
  VisionMarker_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t VisionMarker_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &VisionMarker_message_members,
  get_message_typesupport_handle_function,
  &vdt_msgs__msg__VisionMarker__get_type_hash,
  &vdt_msgs__msg__VisionMarker__get_type_description,
  &vdt_msgs__msg__VisionMarker__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace vdt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<vdt_msgs::msg::VisionMarker>()
{
  return &::vdt_msgs::msg::rosidl_typesupport_introspection_cpp::VisionMarker_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vdt_msgs, msg, VisionMarker)() {
  return &::vdt_msgs::msg::rosidl_typesupport_introspection_cpp::VisionMarker_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
