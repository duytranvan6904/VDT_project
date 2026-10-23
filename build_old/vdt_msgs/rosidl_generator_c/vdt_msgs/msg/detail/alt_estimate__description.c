// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from vdt_msgs:msg/AltEstimate.idl
// generated code does not contain a copyright notice

#include "vdt_msgs/msg/detail/alt_estimate__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
const rosidl_type_hash_t *
vdt_msgs__msg__AltEstimate__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8a, 0x84, 0xa1, 0x69, 0xda, 0xe0, 0x89, 0x77,
      0xd0, 0xc5, 0x60, 0xc0, 0x30, 0x7d, 0x93, 0x2c,
      0x75, 0x56, 0x82, 0xe2, 0xe0, 0xfb, 0x6a, 0x5f,
      0x66, 0x77, 0x64, 0x62, 0xef, 0x20, 0xf6, 0xbd,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char vdt_msgs__msg__AltEstimate__TYPE_NAME[] = "vdt_msgs/msg/AltEstimate";

// Define type names, field names, and default values
static char vdt_msgs__msg__AltEstimate__FIELD_NAME__altitude[] = "altitude";
static char vdt_msgs__msg__AltEstimate__FIELD_NAME__touchdown_flag[] = "touchdown_flag";

static rosidl_runtime_c__type_description__Field vdt_msgs__msg__AltEstimate__FIELDS[] = {
  {
    {vdt_msgs__msg__AltEstimate__FIELD_NAME__altitude, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__AltEstimate__FIELD_NAME__touchdown_flag, 14, 14},
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
vdt_msgs__msg__AltEstimate__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {vdt_msgs__msg__AltEstimate__TYPE_NAME, 24, 24},
      {vdt_msgs__msg__AltEstimate__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 altitude\n"
  "bool touchdown_flag";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
vdt_msgs__msg__AltEstimate__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {vdt_msgs__msg__AltEstimate__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 36, 36},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
vdt_msgs__msg__AltEstimate__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *vdt_msgs__msg__AltEstimate__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
