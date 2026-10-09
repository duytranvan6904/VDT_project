// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from vdt_msgs:msg/VisionMarker.idl
// generated code does not contain a copyright notice
#include "vdt_msgs/msg/detail/vision_marker__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "vdt_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "vdt_msgs/msg/detail/vision_marker__struct.h"
#include "vdt_msgs/msg/detail/vision_marker__functions.h"
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


using _VisionMarker__ros_msg_type = vdt_msgs__msg__VisionMarker;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
bool cdr_serialize_vdt_msgs__msg__VisionMarker(
  const vdt_msgs__msg__VisionMarker * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: marker_visible
  {
    cdr << (ros_message->marker_visible ? true : false);
  }

  // Field name: pixel_align_error
  {
    cdr << ros_message->pixel_align_error;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
bool cdr_deserialize_vdt_msgs__msg__VisionMarker(
  eprosima::fastcdr::Cdr & cdr,
  vdt_msgs__msg__VisionMarker * ros_message)
{
  // Field name: marker_visible
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->marker_visible = tmp ? true : false;
  }

  // Field name: pixel_align_error
  {
    cdr >> ros_message->pixel_align_error;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
size_t get_serialized_size_vdt_msgs__msg__VisionMarker(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _VisionMarker__ros_msg_type * ros_message = static_cast<const _VisionMarker__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: marker_visible
  {
    size_t item_size = sizeof(ros_message->marker_visible);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: pixel_align_error
  {
    size_t item_size = sizeof(ros_message->pixel_align_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
size_t max_serialized_size_vdt_msgs__msg__VisionMarker(
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

  // Field name: marker_visible
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: pixel_align_error
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
    using DataType = vdt_msgs__msg__VisionMarker;
    is_plain =
      (
      offsetof(DataType, pixel_align_error) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
bool cdr_serialize_key_vdt_msgs__msg__VisionMarker(
  const vdt_msgs__msg__VisionMarker * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: marker_visible
  {
    cdr << (ros_message->marker_visible ? true : false);
  }

  // Field name: pixel_align_error
  {
    cdr << ros_message->pixel_align_error;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
size_t get_serialized_size_key_vdt_msgs__msg__VisionMarker(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _VisionMarker__ros_msg_type * ros_message = static_cast<const _VisionMarker__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: marker_visible
  {
    size_t item_size = sizeof(ros_message->marker_visible);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: pixel_align_error
  {
    size_t item_size = sizeof(ros_message->pixel_align_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vdt_msgs
size_t max_serialized_size_key_vdt_msgs__msg__VisionMarker(
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
  // Field name: marker_visible
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: pixel_align_error
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
    using DataType = vdt_msgs__msg__VisionMarker;
    is_plain =
      (
      offsetof(DataType, pixel_align_error) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _VisionMarker__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const vdt_msgs__msg__VisionMarker * ros_message = static_cast<const vdt_msgs__msg__VisionMarker *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_vdt_msgs__msg__VisionMarker(ros_message, cdr);
}

static bool _VisionMarker__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  vdt_msgs__msg__VisionMarker * ros_message = static_cast<vdt_msgs__msg__VisionMarker *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_vdt_msgs__msg__VisionMarker(cdr, ros_message);
}

static uint32_t _VisionMarker__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_vdt_msgs__msg__VisionMarker(
      untyped_ros_message, 0));
}

static size_t _VisionMarker__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_vdt_msgs__msg__VisionMarker(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_VisionMarker = {
  "vdt_msgs::msg",
  "VisionMarker",
  _VisionMarker__cdr_serialize,
  _VisionMarker__cdr_deserialize,
  _VisionMarker__get_serialized_size,
  _VisionMarker__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _VisionMarker__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_VisionMarker,
  get_message_typesupport_handle_function,
  &vdt_msgs__msg__VisionMarker__get_type_hash,
  &vdt_msgs__msg__VisionMarker__get_type_description,
  &vdt_msgs__msg__VisionMarker__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vdt_msgs, msg, VisionMarker)() {
  return &_VisionMarker__type_support;
}

#if defined(__cplusplus)
}
#endif
