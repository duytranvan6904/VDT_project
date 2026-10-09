// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from vdt_msgs:msg/OffboardStatus.idl
// generated code does not contain a copyright notice
#include "vdt_msgs/msg/detail/offboard_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "vdt_msgs/msg/detail/offboard_status__functions.h"
#include "vdt_msgs/msg/detail/offboard_status__struct.hpp"

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
  const vdt_msgs::msg::OffboardStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: offboard_active
  cdr << (ros_message.offboard_active ? true : false);

  // Member: heartbeat_age_sec
  cdr << ros_message.heartbeat_age_sec;

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  vdt_msgs::msg::OffboardStatus & ros_message)
{
  // Member: offboard_active
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.offboard_active = tmp ? true : false;
  }

  // Member: heartbeat_age_sec
  cdr >> ros_message.heartbeat_age_sec;

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
get_serialized_size(
  const vdt_msgs::msg::OffboardStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: offboard_active
  {
    size_t item_size = sizeof(ros_message.offboard_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: heartbeat_age_sec
  {
    size_t item_size = sizeof(ros_message.heartbeat_age_sec);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
max_serialized_size_OffboardStatus(
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

  // Member: offboard_active
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: heartbeat_age_sec
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
    using DataType = vdt_msgs::msg::OffboardStatus;
    is_plain =
      (
      offsetof(DataType, heartbeat_age_sec) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
cdr_serialize_key(
  const vdt_msgs::msg::OffboardStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: offboard_active
  cdr << (ros_message.offboard_active ? true : false);

  // Member: heartbeat_age_sec
  cdr << ros_message.heartbeat_age_sec;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
get_serialized_size_key(
  const vdt_msgs::msg::OffboardStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: offboard_active
  {
    size_t item_size = sizeof(ros_message.offboard_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: heartbeat_age_sec
  {
    size_t item_size = sizeof(ros_message.heartbeat_age_sec);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vdt_msgs
max_serialized_size_key_OffboardStatus(
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

  // Member: offboard_active
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: heartbeat_age_sec
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
    using DataType = vdt_msgs::msg::OffboardStatus;
    is_plain =
      (
      offsetof(DataType, heartbeat_age_sec) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}


static bool _OffboardStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const vdt_msgs::msg::OffboardStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _OffboardStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<vdt_msgs::msg::OffboardStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _OffboardStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const vdt_msgs::msg::OffboardStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _OffboardStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_OffboardStatus(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _OffboardStatus__callbacks = {
  "vdt_msgs::msg",
  "OffboardStatus",
  _OffboardStatus__cdr_serialize,
  _OffboardStatus__cdr_deserialize,
  _OffboardStatus__get_serialized_size,
  _OffboardStatus__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _OffboardStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_OffboardStatus__callbacks,
  get_message_typesupport_handle_function,
  &vdt_msgs__msg__OffboardStatus__get_type_hash,
  &vdt_msgs__msg__OffboardStatus__get_type_description,
  &vdt_msgs__msg__OffboardStatus__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace vdt_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_vdt_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<vdt_msgs::msg::OffboardStatus>()
{
  return &vdt_msgs::msg::typesupport_fastrtps_cpp::_OffboardStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vdt_msgs, msg, OffboardStatus)() {
  return &vdt_msgs::msg::typesupport_fastrtps_cpp::_OffboardStatus__handle;
}

#ifdef __cplusplus
}
#endif
