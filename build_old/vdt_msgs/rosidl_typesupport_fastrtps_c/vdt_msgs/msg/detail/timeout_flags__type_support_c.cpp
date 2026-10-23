// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from vdt_msgs:msg/TimeoutFlags.idl
// generated code does not contain a copyright notice
#include "vdt_msgs/msg/detail/timeout_flags__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "vdt_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "vdt_msgs/msg/detail/timeout_flags__struct.h"
#include "vdt_msgs/msg/detail/timeout_flags__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _TimeoutFlags__ros_msg_type = vdt_msgs__msg__TimeoutFlags;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
bool cdr_serialize_vdt_msgs__msg__TimeoutFlags(
  const vdt_msgs__msg__TimeoutFlags * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: ekf_timeout
  {
    cdr << (ros_message->ekf_timeout ? true : false);
  }

  // Field name: vision_timeout
  {
    cdr << (ros_message->vision_timeout ? true : false);
  }

  // Field name: alt_timeout
  {
    cdr << (ros_message->alt_timeout ? true : false);
  }

  // Field name: planner_timeout
  {
    cdr << (ros_message->planner_timeout ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
bool cdr_deserialize_vdt_msgs__msg__TimeoutFlags(
  eprosima::fastcdr::Cdr & cdr,
  vdt_msgs__msg__TimeoutFlags * ros_message)
{
  // Field name: ekf_timeout
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->ekf_timeout = tmp ? true : false;
  }

  // Field name: vision_timeout
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->vision_timeout = tmp ? true : false;
  }

  // Field name: alt_timeout
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->alt_timeout = tmp ? true : false;
  }

  // Field name: planner_timeout
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->planner_timeout = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
size_t get_serialized_size_vdt_msgs__msg__TimeoutFlags(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TimeoutFlags__ros_msg_type * ros_message = static_cast<const _TimeoutFlags__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: ekf_timeout
  {
    size_t item_size = sizeof(ros_message->ekf_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: vision_timeout
  {
    size_t item_size = sizeof(ros_message->vision_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: alt_timeout
  {
    size_t item_size = sizeof(ros_message->alt_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: planner_timeout
  {
    size_t item_size = sizeof(ros_message->planner_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
size_t max_serialized_size_vdt_msgs__msg__TimeoutFlags(
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

  // Field name: ekf_timeout
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: vision_timeout
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: alt_timeout
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: planner_timeout
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
    using DataType = vdt_msgs__msg__TimeoutFlags;
    is_plain =
      (
      offsetof(DataType, planner_timeout) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
bool cdr_serialize_key_vdt_msgs__msg__TimeoutFlags(
  const vdt_msgs__msg__TimeoutFlags * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: ekf_timeout
  {
    cdr << (ros_message->ekf_timeout ? true : false);
  }

  // Field name: vision_timeout
  {
    cdr << (ros_message->vision_timeout ? true : false);
  }

  // Field name: alt_timeout
  {
    cdr << (ros_message->alt_timeout ? true : false);
  }

  // Field name: planner_timeout
  {
    cdr << (ros_message->planner_timeout ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
size_t get_serialized_size_key_vdt_msgs__msg__TimeoutFlags(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TimeoutFlags__ros_msg_type * ros_message = static_cast<const _TimeoutFlags__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: ekf_timeout
  {
    size_t item_size = sizeof(ros_message->ekf_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: vision_timeout
  {
    size_t item_size = sizeof(ros_message->vision_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: alt_timeout
  {
    size_t item_size = sizeof(ros_message->alt_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: planner_timeout
  {
    size_t item_size = sizeof(ros_message->planner_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
size_t max_serialized_size_key_vdt_msgs__msg__TimeoutFlags(
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
  // Field name: ekf_timeout
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: vision_timeout
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: alt_timeout
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: planner_timeout
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
    using DataType = vdt_msgs__msg__TimeoutFlags;
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
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const vdt_msgs__msg__TimeoutFlags * ros_message = static_cast<const vdt_msgs__msg__TimeoutFlags *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_vdt_msgs__msg__TimeoutFlags(ros_message, cdr);
}

static bool _TimeoutFlags__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  vdt_msgs__msg__TimeoutFlags * ros_message = static_cast<vdt_msgs__msg__TimeoutFlags *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_vdt_msgs__msg__TimeoutFlags(cdr, ros_message);
}

static uint32_t _TimeoutFlags__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_vdt_msgs__msg__TimeoutFlags(
      untyped_ros_message, 0));
}

static size_t _TimeoutFlags__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_vdt_msgs__msg__TimeoutFlags(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_TimeoutFlags = {
  "vdt_msgs::msg",
  "TimeoutFlags",
  _TimeoutFlags__cdr_serialize,
  _TimeoutFlags__cdr_deserialize,
  _TimeoutFlags__get_serialized_size,
  _TimeoutFlags__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _TimeoutFlags__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TimeoutFlags,
  get_message_typesupport_handle_function,
  &vdt_msgs__msg__TimeoutFlags__get_type_hash,
  &vdt_msgs__msg__TimeoutFlags__get_type_description,
  &vdt_msgs__msg__TimeoutFlags__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vdt_msgs, msg, TimeoutFlags)() {
  return &_TimeoutFlags__type_support;
}

#if defined(__cplusplus)
}
#endif
