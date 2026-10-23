// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from vdt_msgs:msg/InputSnapshot.idl
// generated code does not contain a copyright notice

#include "vdt_msgs/msg/detail/input_snapshot__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
const rosidl_type_hash_t *
vdt_msgs__msg__InputSnapshot__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf7, 0x85, 0x4f, 0x00, 0x4b, 0x80, 0xd7, 0x5f,
      0x9f, 0xac, 0xc0, 0xd0, 0xa7, 0xd0, 0x8f, 0x9e,
      0x11, 0x1b, 0xe6, 0x70, 0x78, 0xbf, 0xa1, 0x37,
      0xde, 0x90, 0xe3, 0x92, 0xcc, 0xbc, 0x6c, 0xf0,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char vdt_msgs__msg__InputSnapshot__TYPE_NAME[] = "vdt_msgs/msg/InputSnapshot";

// Define type names, field names, and default values
static char vdt_msgs__msg__InputSnapshot__FIELD_NAME__valid[] = "valid";
static char vdt_msgs__msg__InputSnapshot__FIELD_NAME__marker_detected[] = "marker_detected";
static char vdt_msgs__msg__InputSnapshot__FIELD_NAME__align_error[] = "align_error";
static char vdt_msgs__msg__InputSnapshot__FIELD_NAME__altitude[] = "altitude";
static char vdt_msgs__msg__InputSnapshot__FIELD_NAME__delta_h[] = "delta_h";
static char vdt_msgs__msg__InputSnapshot__FIELD_NAME__d_horiz[] = "d_horiz";
static char vdt_msgs__msg__InputSnapshot__FIELD_NAME__touchdown[] = "touchdown";
static char vdt_msgs__msg__InputSnapshot__FIELD_NAME__planner_timeout[] = "planner_timeout";
static char vdt_msgs__msg__InputSnapshot__FIELD_NAME__yaw_rate[] = "yaw_rate";

static rosidl_runtime_c__type_description__Field vdt_msgs__msg__InputSnapshot__FIELDS[] = {
  {
    {vdt_msgs__msg__InputSnapshot__FIELD_NAME__valid, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__InputSnapshot__FIELD_NAME__marker_detected, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__InputSnapshot__FIELD_NAME__align_error, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__InputSnapshot__FIELD_NAME__altitude, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__InputSnapshot__FIELD_NAME__delta_h, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__InputSnapshot__FIELD_NAME__d_horiz, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__InputSnapshot__FIELD_NAME__touchdown, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__InputSnapshot__FIELD_NAME__planner_timeout, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__InputSnapshot__FIELD_NAME__yaw_rate, 8, 8},
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
vdt_msgs__msg__InputSnapshot__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {vdt_msgs__msg__InputSnapshot__TYPE_NAME, 26, 26},
      {vdt_msgs__msg__InputSnapshot__FIELDS, 9, 9},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool valid\n"
  "bool marker_detected\n"
  "float32 align_error\n"
  "float32 altitude\n"
  "float32 delta_h\n"
  "float32 d_horiz\n"
  "bool touchdown\n"
  "bool planner_timeout\n"
  "float32 yaw_rate";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
vdt_msgs__msg__InputSnapshot__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {vdt_msgs__msg__InputSnapshot__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 153, 153},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
vdt_msgs__msg__InputSnapshot__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *vdt_msgs__msg__InputSnapshot__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
