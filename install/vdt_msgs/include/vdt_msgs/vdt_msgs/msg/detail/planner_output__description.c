// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from vdt_msgs:msg/PlannerOutput.idl
// generated code does not contain a copyright notice

#include "vdt_msgs/msg/detail/planner_output__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
const rosidl_type_hash_t *
vdt_msgs__msg__PlannerOutput__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x93, 0x5a, 0x14, 0xb7, 0x10, 0xc5, 0xa0, 0x38,
      0xfc, 0xe5, 0x0a, 0x9a, 0x01, 0x56, 0xc1, 0x09,
      0x5a, 0xde, 0x38, 0x41, 0xf2, 0x75, 0xb4, 0xda,
      0x34, 0x8b, 0xed, 0x66, 0x0b, 0x53, 0x88, 0x15,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char vdt_msgs__msg__PlannerOutput__TYPE_NAME[] = "vdt_msgs/msg/PlannerOutput";

// Define type names, field names, and default values
static char vdt_msgs__msg__PlannerOutput__FIELD_NAME__vx[] = "vx";
static char vdt_msgs__msg__PlannerOutput__FIELD_NAME__vy[] = "vy";
static char vdt_msgs__msg__PlannerOutput__FIELD_NAME__vz[] = "vz";
static char vdt_msgs__msg__PlannerOutput__FIELD_NAME__yaw[] = "yaw";

static rosidl_runtime_c__type_description__Field vdt_msgs__msg__PlannerOutput__FIELDS[] = {
  {
    {vdt_msgs__msg__PlannerOutput__FIELD_NAME__vx, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__PlannerOutput__FIELD_NAME__vy, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__PlannerOutput__FIELD_NAME__vz, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__PlannerOutput__FIELD_NAME__yaw, 3, 3},
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
vdt_msgs__msg__PlannerOutput__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {vdt_msgs__msg__PlannerOutput__TYPE_NAME, 26, 26},
      {vdt_msgs__msg__PlannerOutput__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 vx\n"
  "float32 vy\n"
  "float32 vz\n"
  "float32 yaw";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
vdt_msgs__msg__PlannerOutput__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {vdt_msgs__msg__PlannerOutput__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 44, 44},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
vdt_msgs__msg__PlannerOutput__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *vdt_msgs__msg__PlannerOutput__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
