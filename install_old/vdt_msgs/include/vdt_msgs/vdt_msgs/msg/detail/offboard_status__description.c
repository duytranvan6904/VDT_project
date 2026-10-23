// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from vdt_msgs:msg/OffboardStatus.idl
// generated code does not contain a copyright notice

#include "vdt_msgs/msg/detail/offboard_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
const rosidl_type_hash_t *
vdt_msgs__msg__OffboardStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x10, 0x20, 0xfa, 0x2d, 0xd7, 0x68, 0x99, 0x34,
      0x61, 0x6c, 0xe0, 0x69, 0xd0, 0x32, 0x9e, 0x0d,
      0xee, 0xa2, 0x08, 0x3c, 0x08, 0x3d, 0x13, 0x06,
      0xe3, 0xca, 0xdd, 0xcb, 0x79, 0x18, 0xbc, 0xbd,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char vdt_msgs__msg__OffboardStatus__TYPE_NAME[] = "vdt_msgs/msg/OffboardStatus";

// Define type names, field names, and default values
static char vdt_msgs__msg__OffboardStatus__FIELD_NAME__offboard_active[] = "offboard_active";
static char vdt_msgs__msg__OffboardStatus__FIELD_NAME__heartbeat_age_sec[] = "heartbeat_age_sec";

static rosidl_runtime_c__type_description__Field vdt_msgs__msg__OffboardStatus__FIELDS[] = {
  {
    {vdt_msgs__msg__OffboardStatus__FIELD_NAME__offboard_active, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__OffboardStatus__FIELD_NAME__heartbeat_age_sec, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
vdt_msgs__msg__OffboardStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {vdt_msgs__msg__OffboardStatus__TYPE_NAME, 27, 27},
      {vdt_msgs__msg__OffboardStatus__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool offboard_active\n"
  "float32 heartbeat_age_sec";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
vdt_msgs__msg__OffboardStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {vdt_msgs__msg__OffboardStatus__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 46, 46},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
vdt_msgs__msg__OffboardStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *vdt_msgs__msg__OffboardStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
