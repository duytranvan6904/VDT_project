// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from vdt_msgs:msg/VisionMarker.idl
// generated code does not contain a copyright notice

#include "vdt_msgs/msg/detail/vision_marker__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
const rosidl_type_hash_t *
vdt_msgs__msg__VisionMarker__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7d, 0x8d, 0xce, 0xee, 0x73, 0x47, 0x29, 0xf8,
      0xbc, 0x06, 0x3d, 0x25, 0x0c, 0x23, 0x17, 0xb4,
      0xd1, 0xc9, 0xf1, 0x88, 0xce, 0x81, 0xd1, 0x57,
      0x11, 0x61, 0x21, 0xd2, 0x08, 0xd8, 0x3f, 0x11,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char vdt_msgs__msg__VisionMarker__TYPE_NAME[] = "vdt_msgs/msg/VisionMarker";

// Define type names, field names, and default values
static char vdt_msgs__msg__VisionMarker__FIELD_NAME__marker_visible[] = "marker_visible";
static char vdt_msgs__msg__VisionMarker__FIELD_NAME__pixel_align_error[] = "pixel_align_error";

static rosidl_runtime_c__type_description__Field vdt_msgs__msg__VisionMarker__FIELDS[] = {
  {
    {vdt_msgs__msg__VisionMarker__FIELD_NAME__marker_visible, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vdt_msgs__msg__VisionMarker__FIELD_NAME__pixel_align_error, 17, 17},
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
vdt_msgs__msg__VisionMarker__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {vdt_msgs__msg__VisionMarker__TYPE_NAME, 25, 25},
      {vdt_msgs__msg__VisionMarker__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool marker_visible\n"
  "float32 pixel_align_error";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
vdt_msgs__msg__VisionMarker__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {vdt_msgs__msg__VisionMarker__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 45, 45},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
vdt_msgs__msg__VisionMarker__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *vdt_msgs__msg__VisionMarker__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
