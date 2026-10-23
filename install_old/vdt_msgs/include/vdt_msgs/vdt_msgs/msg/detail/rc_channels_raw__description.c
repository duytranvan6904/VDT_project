// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from vdt_msgs:msg/RcChannelsRaw.idl
// generated code does not contain a copyright notice

#include "vdt_msgs/msg/detail/rc_channels_raw__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
const rosidl_type_hash_t *
vdt_msgs__msg__RcChannelsRaw__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5b, 0xd1, 0xc9, 0xc8, 0xb0, 0x92, 0xa0, 0xca,
      0x1c, 0x67, 0x99, 0x9e, 0xa7, 0xa0, 0x59, 0x53,
      0x4b, 0x0c, 0xc4, 0xae, 0x03, 0x2f, 0x92, 0x84,
      0xad, 0x2b, 0x10, 0xc5, 0xfe, 0x50, 0xf9, 0x59,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char vdt_msgs__msg__RcChannelsRaw__TYPE_NAME[] = "vdt_msgs/msg/RcChannelsRaw";

// Define type names, field names, and default values
static char vdt_msgs__msg__RcChannelsRaw__FIELD_NAME__ch[] = "ch";
static char vdt_msgs__msg__RcChannelsRaw__FIELD_NAME__valid[] = "valid";
static char vdt_msgs__msg__RcChannelsRaw__FIELD_NAME__failsafe[] = "failsafe";

static rosidl_runtime_c__type_description__Field vdt_msgs__msg__RcChannelsRaw__FIELDS[] = {
  {
    {vdt_msgs__msg__RcChannelsRaw__FIELD_NAME__ch, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16_ARRAY,
      16,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__RcChannelsRaw__FIELD_NAME__valid, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__RcChannelsRaw__FIELD_NAME__failsafe, 8, 8},
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
vdt_msgs__msg__RcChannelsRaw__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {vdt_msgs__msg__RcChannelsRaw__TYPE_NAME, 26, 26},
      {vdt_msgs__msg__RcChannelsRaw__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int16[16] ch\n"
  "bool valid\n"
  "bool failsafe";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
vdt_msgs__msg__RcChannelsRaw__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {vdt_msgs__msg__RcChannelsRaw__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 37, 37},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
vdt_msgs__msg__RcChannelsRaw__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *vdt_msgs__msg__RcChannelsRaw__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
