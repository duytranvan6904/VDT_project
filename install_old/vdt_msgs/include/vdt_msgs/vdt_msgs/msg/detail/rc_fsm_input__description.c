// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from vdt_msgs:msg/RcFsmInput.idl
// generated code does not contain a copyright notice

#include "vdt_msgs/msg/detail/rc_fsm_input__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
const rosidl_type_hash_t *
vdt_msgs__msg__RcFsmInput__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3f, 0x29, 0xf0, 0x1d, 0x15, 0x5b, 0x86, 0x73,
      0xfd, 0x72, 0x1f, 0x73, 0xfe, 0x76, 0xa1, 0x3c,
      0x0f, 0x1c, 0x35, 0xf3, 0x44, 0x7a, 0x4a, 0x01,
      0x04, 0x6c, 0xfc, 0xbc, 0xbd, 0x28, 0xcd, 0x3b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char vdt_msgs__msg__RcFsmInput__TYPE_NAME[] = "vdt_msgs/msg/RcFsmInput";

// Define type names, field names, and default values
static char vdt_msgs__msg__RcFsmInput__FIELD_NAME__land_switch[] = "land_switch";
static char vdt_msgs__msg__RcFsmInput__FIELD_NAME__kill_switch[] = "kill_switch";

static rosidl_runtime_c__type_description__Field vdt_msgs__msg__RcFsmInput__FIELDS[] = {
  {
    {vdt_msgs__msg__RcFsmInput__FIELD_NAME__land_switch, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__RcFsmInput__FIELD_NAME__kill_switch, 11, 11},
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
vdt_msgs__msg__RcFsmInput__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {vdt_msgs__msg__RcFsmInput__TYPE_NAME, 23, 23},
      {vdt_msgs__msg__RcFsmInput__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool land_switch\n"
  "bool kill_switch";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
vdt_msgs__msg__RcFsmInput__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {vdt_msgs__msg__RcFsmInput__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 33, 33},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
vdt_msgs__msg__RcFsmInput__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *vdt_msgs__msg__RcFsmInput__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
