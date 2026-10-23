// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from vdt_msgs:msg/TimeoutFlags.idl
// generated code does not contain a copyright notice

#include "vdt_msgs/msg/detail/timeout_flags__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
const rosidl_type_hash_t *
vdt_msgs__msg__TimeoutFlags__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2a, 0x8b, 0x01, 0xe2, 0x8d, 0x67, 0x8c, 0x85,
      0x7d, 0x9b, 0xe2, 0xd2, 0x48, 0xa6, 0x2f, 0xc0,
      0xef, 0x2c, 0xac, 0x6e, 0x8f, 0x56, 0x97, 0xb1,
      0x59, 0x4a, 0xb2, 0x7c, 0xc7, 0x6c, 0xe5, 0xde,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char vdt_msgs__msg__TimeoutFlags__TYPE_NAME[] = "vdt_msgs/msg/TimeoutFlags";

// Define type names, field names, and default values
static char vdt_msgs__msg__TimeoutFlags__FIELD_NAME__ekf_timeout[] = "ekf_timeout";
static char vdt_msgs__msg__TimeoutFlags__FIELD_NAME__vision_timeout[] = "vision_timeout";
static char vdt_msgs__msg__TimeoutFlags__FIELD_NAME__alt_timeout[] = "alt_timeout";
static char vdt_msgs__msg__TimeoutFlags__FIELD_NAME__planner_timeout[] = "planner_timeout";

static rosidl_runtime_c__type_description__Field vdt_msgs__msg__TimeoutFlags__FIELDS[] = {
  {
    {vdt_msgs__msg__TimeoutFlags__FIELD_NAME__ekf_timeout, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__TimeoutFlags__FIELD_NAME__vision_timeout, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__TimeoutFlags__FIELD_NAME__alt_timeout, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__TimeoutFlags__FIELD_NAME__planner_timeout, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
vdt_msgs__msg__TimeoutFlags__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {vdt_msgs__msg__TimeoutFlags__TYPE_NAME, 25, 25},
      {vdt_msgs__msg__TimeoutFlags__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool ekf_timeout\n"
  "bool vision_timeout\n"
  "bool alt_timeout\n"
  "bool planner_timeout";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
vdt_msgs__msg__TimeoutFlags__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {vdt_msgs__msg__TimeoutFlags__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 74, 74},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
vdt_msgs__msg__TimeoutFlags__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *vdt_msgs__msg__TimeoutFlags__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
