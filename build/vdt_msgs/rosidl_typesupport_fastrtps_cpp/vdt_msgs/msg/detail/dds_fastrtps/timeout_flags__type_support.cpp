// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from vdt_msgs:msg/TimeoutFlags.idl
// generated code does not contain a copyright notice
#include "vdt_msgs/msg/detail/timeout_flags__rosidl_typesupport_fastrtps_cpp.hpp"
#include "vdt_msgs/msg/detail/timeout_flags__functions.h"
#include "vdt_msgs/msg/detail/timeout_flags__struct.hpp"

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace vdt_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
cdr_serialize(
  const vdt_msgs::msg::TimeoutFlags & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: ekf_timeout
  cdr << (ros_message.ekf_timeout ? true : false);

  // Member: vision_timeout
  cdr << (ros_message.vision_timeout ? true : false);

  // Member: alt_timeout
  cdr << (ros_message.alt_timeout ? true : false);

  // Member: planner_timeout
  cdr << (ros_message.planner_timeout ? true : false);

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  vdt_msgs::msg::TimeoutFlags & ros_message)
{
  // Member: ekf_timeout
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.ekf_timeout = tmp ? true : false;
  }

  // Member: vision_timeout
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.vision_timeout = tmp ? true : false;
  }

  // Member: alt_timeout
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.alt_timeout = tmp ? true : false;
  }

  // Member: planner_timeout
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.planner_timeout = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
get_serialized_size(
  const vdt_msgs::msg::TimeoutFlags & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: ekf_timeout
  {
    size_t item_size = sizeof(ros_message.ekf_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vision_timeout
  {
    size_t item_size = sizeof(ros_message.vision_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: alt_timeout
  {
    size_t item_size = sizeof(ros_message.alt_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: planner_timeout
  {
    size_t item_size = sizeof(ros_message.planner_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
max_serialized_size_TimeoutFlags(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Member: ekf_timeout
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: vision_timeout
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: alt_timeout
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: planner_timeout
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vdt_msgs::msg::TimeoutFlags;
    is_plain =
      (
      offsetof(DataType, planner_timeout) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
cdr_serialize_key(
  const vdt_msgs::msg::TimeoutFlags & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: ekf_timeout
  cdr << (ros_message.ekf_timeout ? true : false);

  // Member: vision_timeout
  cdr << (ros_message.vision_timeout ? true : false);

  // Member: alt_timeout
  cdr << (ros_message.alt_timeout ? true : false);

  // Member: planner_timeout
  cdr << (ros_message.planner_timeout ? true : false);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
get_serialized_size_key(
  const vdt_msgs::msg::TimeoutFlags & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: ekf_timeout
  {
    size_t item_size = sizeof(ros_message.ekf_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vision_timeout
  {
    size_t item_size = sizeof(ros_message.vision_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: alt_timeout
  {
    size_t item_size = sizeof(ros_message.alt_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: planner_timeout
  {
    size_t item_size = sizeof(ros_message.planner_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
max_serialized_size_key_TimeoutFlags(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Member: ekf_timeout
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: vision_timeout
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: alt_timeout
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: planner_timeout
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vdt_msgs::msg::TimeoutFlags;
    is_plain =
      (
      offsetof(DataType, planner_timeout) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}


static bool _TimeoutFlags__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const vdt_msgs::msg::TimeoutFlags *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _TimeoutFlags__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<vdt_msgs::msg::TimeoutFlags *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _TimeoutFlags__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const vdt_msgs::msg::TimeoutFlags *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _TimeoutFlags__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_TimeoutFlags(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _TimeoutFlags__callbacks = {
  "vdt_msgs::msg",
  "TimeoutFlags",
  _TimeoutFlags__cdr_serialize,
  _TimeoutFlags__cdr_deserialize,
  _TimeoutFlags__get_serialized_size,
  _TimeoutFlags__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _TimeoutFlags__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_TimeoutFlags__callbacks,
  get_message_typesupport_handle_function,
  &vdt_msgs__msg__TimeoutFlags__get_type_hash,
  &vdt_msgs__msg__TimeoutFlags__get_type_description,
  &vdt_msgs__msg__TimeoutFlags__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace vdt_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_vdt_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<vdt_msgs::msg::TimeoutFlags>()
{
  return &vdt_msgs::msg::typesupport_fastrtps_cpp::_TimeoutFlags__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vdt_msgs, msg, TimeoutFlags)() {
  return &vdt_msgs::msg::typesupport_fastrtps_cpp::_TimeoutFlags__handle;
}

#ifdef __cplusplus
}
#endif
