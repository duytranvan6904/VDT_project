// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from vdt_msgs:msg/RcChannelsRaw.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "vdt_msgs/msg/detail/rc_channels_raw__functions.h"
#include "vdt_msgs/msg/detail/rc_channels_raw__struct.hpp"
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

void RcChannelsRaw_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) vdt_msgs::msg::RcChannelsRaw(_init);
}

void RcChannelsRaw_fini_function(void * message_memory)
{
  auto typed_message = static_cast<vdt_msgs::msg::RcChannelsRaw *>(message_memory);
  typed_message->~RcChannelsRaw();
}

size_t size_function__RcChannelsRaw__ch(const void * untyped_member)
{
  (void)untyped_member;
  return 16;
}

const void * get_const_function__RcChannelsRaw__ch(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int16_t, 16> *>(untyped_member);
  return &member[index];
}

void * get_function__RcChannelsRaw__ch(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int16_t, 16> *>(untyped_member);
  return &member[index];
}

void fetch_function__RcChannelsRaw__ch(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int16_t *>(
    get_const_function__RcChannelsRaw__ch(untyped_member, index));
  auto & value = *reinterpret_cast<int16_t *>(untyped_value);
  value = item;
}

void assign_function__RcChannelsRaw__ch(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int16_t *>(
    get_function__RcChannelsRaw__ch(untyped_member, index));
  const auto & value = *reinterpret_cast<const int16_t *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RcChannelsRaw_message_member_array[3] = {
  {
    "ch",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    16,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs::msg::RcChannelsRaw, ch),  // bytes offset in struct
    nullptr,  // default value
    size_function__RcChannelsRaw__ch,  // size() function pointer
    get_const_function__RcChannelsRaw__ch,  // get_const(index) function pointer
    get_function__RcChannelsRaw__ch,  // get(index) function pointer
    fetch_function__RcChannelsRaw__ch,  // fetch(index, &value) function pointer
    assign_function__RcChannelsRaw__ch,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "valid",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs::msg::RcChannelsRaw, valid),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "failsafe",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vdt_msgs::msg::RcChannelsRaw, failsafe),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RcChannelsRaw_message_members = {
  "vdt_msgs::msg",  // message namespace
  "RcChannelsRaw",  // message name
  3,  // number of fields
  sizeof(vdt_msgs::msg::RcChannelsRaw),
  false,  // has_any_key_member_
  RcChannelsRaw_message_member_array,  // message members
  RcChannelsRaw_init_function,  // function to initialize message memory (memory has to be allocated)
  RcChannelsRaw_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RcChannelsRaw_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RcChannelsRaw_message_members,
  get_message_typesupport_handle_function,
  &vdt_msgs__msg__RcChannelsRaw__get_type_hash,
  &vdt_msgs__msg__RcChannelsRaw__get_type_description,
  &vdt_msgs__msg__RcChannelsRaw__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace vdt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<vdt_msgs::msg::RcChannelsRaw>()
{
  return &::vdt_msgs::msg::rosidl_typesupport_introspection_cpp::RcChannelsRaw_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vdt_msgs, msg, RcChannelsRaw)() {
  return &::vdt_msgs::msg::rosidl_typesupport_introspection_cpp::RcChannelsRaw_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
