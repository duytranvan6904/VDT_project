// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from vdt_msgs:msg/InputSnapshot.idl
// generated code does not contain a copyright notice
#include "vdt_msgs/msg/detail/input_snapshot__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "vdt_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "vdt_msgs/msg/detail/input_snapshot__struct.h"
#include "vdt_msgs/msg/detail/input_snapshot__functions.h"
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


using _InputSnapshot__ros_msg_type = vdt_msgs__msg__InputSnapshot;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
bool cdr_serialize_vdt_msgs__msg__InputSnapshot(
  const vdt_msgs__msg__InputSnapshot * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: valid
  {
    cdr << (ros_message->valid ? true : false);
  }

  // Field name: marker_detected
  {
    cdr << (ros_message->marker_detected ? true : false);
  }

  // Field name: align_error
  {
    cdr << ros_message->align_error;
  }

  // Field name: altitude
  {
    cdr << ros_message->altitude;
  }

  // Field name: delta_h
  {
    cdr << ros_message->delta_h;
  }

  // Field name: d_horiz
  {
    cdr << ros_message->d_horiz;
  }

  // Field name: touchdown
  {
    cdr << (ros_message->touchdown ? true : false);
  }

  // Field name: planner_timeout
  {
    cdr << (ros_message->planner_timeout ? true : false);
  }

  // Field name: yaw_rate
  {
    cdr << ros_message->yaw_rate;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
bool cdr_deserialize_vdt_msgs__msg__InputSnapshot(
  eprosima::fastcdr::Cdr & cdr,
  vdt_msgs__msg__InputSnapshot * ros_message)
{
  // Field name: valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->valid = tmp ? true : false;
  }

  // Field name: marker_detected
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->marker_detected = tmp ? true : false;
  }

  // Field name: align_error
  {
    cdr >> ros_message->align_error;
  }

  // Field name: altitude
  {
    cdr >> ros_message->altitude;
  }

  // Field name: delta_h
  {
    cdr >> ros_message->delta_h;
  }

  // Field name: d_horiz
  {
    cdr >> ros_message->d_horiz;
  }

  // Field name: touchdown
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->touchdown = tmp ? true : false;
  }

  // Field name: planner_timeout
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->planner_timeout = tmp ? true : false;
  }

  // Field name: yaw_rate
  {
    cdr >> ros_message->yaw_rate;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
size_t get_serialized_size_vdt_msgs__msg__InputSnapshot(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _InputSnapshot__ros_msg_type * ros_message = static_cast<const _InputSnapshot__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: valid
  {
    size_t item_size = sizeof(ros_message->valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: marker_detected
  {
    size_t item_size = sizeof(ros_message->marker_detected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: align_error
  {
    size_t item_size = sizeof(ros_message->align_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: altitude
  {
    size_t item_size = sizeof(ros_message->altitude);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: delta_h
  {
    size_t item_size = sizeof(ros_message->delta_h);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: d_horiz
  {
    size_t item_size = sizeof(ros_message->d_horiz);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: touchdown
  {
    size_t item_size = sizeof(ros_message->touchdown);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: planner_timeout
  {
    size_t item_size = sizeof(ros_message->planner_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: yaw_rate
  {
    size_t item_size = sizeof(ros_message->yaw_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
size_t max_serialized_size_vdt_msgs__msg__InputSnapshot(
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

  // Field name: valid
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: marker_detected
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: align_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: altitude
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: delta_h
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: d_horiz
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: touchdown
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

  // Field name: yaw_rate
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vdt_msgs__msg__InputSnapshot;
    is_plain =
      (
      offsetof(DataType, yaw_rate) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
bool cdr_serialize_key_vdt_msgs__msg__InputSnapshot(
  const vdt_msgs__msg__InputSnapshot * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: valid
  {
    cdr << (ros_message->valid ? true : false);
  }

  // Field name: marker_detected
  {
    cdr << (ros_message->marker_detected ? true : false);
  }

  // Field name: align_error
  {
    cdr << ros_message->align_error;
  }

  // Field name: altitude
  {
    cdr << ros_message->altitude;
  }

  // Field name: delta_h
  {
    cdr << ros_message->delta_h;
  }

  // Field name: d_horiz
  {
    cdr << ros_message->d_horiz;
  }

  // Field name: touchdown
  {
    cdr << (ros_message->touchdown ? true : false);
  }

  // Field name: planner_timeout
  {
    cdr << (ros_message->planner_timeout ? true : false);
  }

  // Field name: yaw_rate
  {
    cdr << ros_message->yaw_rate;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
size_t get_serialized_size_key_vdt_msgs__msg__InputSnapshot(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _InputSnapshot__ros_msg_type * ros_message = static_cast<const _InputSnapshot__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: valid
  {
    size_t item_size = sizeof(ros_message->valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: marker_detected
  {
    size_t item_size = sizeof(ros_message->marker_detected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: align_error
  {
    size_t item_size = sizeof(ros_message->align_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: altitude
  {
    size_t item_size = sizeof(ros_message->altitude);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: delta_h
  {
    size_t item_size = sizeof(ros_message->delta_h);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: d_horiz
  {
    size_t item_size = sizeof(ros_message->d_horiz);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: touchdown
  {
    size_t item_size = sizeof(ros_message->touchdown);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: planner_timeout
  {
    size_t item_size = sizeof(ros_message->planner_timeout);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: yaw_rate
  {
    size_t item_size = sizeof(ros_message->yaw_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
size_t max_serialized_size_key_vdt_msgs__msg__InputSnapshot(
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
  // Field name: valid
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: marker_detected
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: align_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: altitude
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: delta_h
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: d_horiz
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: touchdown
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

  // Field name: yaw_rate
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vdt_msgs__msg__InputSnapshot;
    is_plain =
      (
      offsetof(DataType, yaw_rate) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _InputSnapshot__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const vdt_msgs__msg__InputSnapshot * ros_message = static_cast<const vdt_msgs__msg__InputSnapshot *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_vdt_msgs__msg__InputSnapshot(ros_message, cdr);
}

static bool _InputSnapshot__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  vdt_msgs__msg__InputSnapshot * ros_message = static_cast<vdt_msgs__msg__InputSnapshot *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_vdt_msgs__msg__InputSnapshot(cdr, ros_message);
}

static uint32_t _InputSnapshot__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_vdt_msgs__msg__InputSnapshot(
      untyped_ros_message, 0));
}

static size_t _InputSnapshot__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_vdt_msgs__msg__InputSnapshot(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_InputSnapshot = {
  "vdt_msgs::msg",
  "InputSnapshot",
  _InputSnapshot__cdr_serialize,
  _InputSnapshot__cdr_deserialize,
  _InputSnapshot__get_serialized_size,
  _InputSnapshot__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _InputSnapshot__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_InputSnapshot,
  get_message_typesupport_handle_function,
  &vdt_msgs__msg__InputSnapshot__get_type_hash,
  &vdt_msgs__msg__InputSnapshot__get_type_description,
  &vdt_msgs__msg__InputSnapshot__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vdt_msgs, msg, InputSnapshot)() {
  return &_InputSnapshot__type_support;
}

#if defined(__cplusplus)
}
#endif
