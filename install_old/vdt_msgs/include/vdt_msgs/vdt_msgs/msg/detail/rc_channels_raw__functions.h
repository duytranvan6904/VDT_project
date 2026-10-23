// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from vdt_msgs:msg/RcChannelsRaw.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vdt_msgs/msg/rc_channels_raw.h"


#ifndef VDT_MSGS__MSG__DETAIL__RC_CHANNELS_RAW__FUNCTIONS_H_
#define VDT_MSGS__MSG__DETAIL__RC_CHANNELS_RAW__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "vdt_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "vdt_msgs/msg/detail/rc_channels_raw__struct.h"

/// Initialize msg/RcChannelsRaw message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * vdt_msgs__msg__RcChannelsRaw
 * )) before or use
 * vdt_msgs__msg__RcChannelsRaw__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
bool
vdt_msgs__msg__RcChannelsRaw__init(vdt_msgs__msg__RcChannelsRaw * msg);

/// Finalize msg/RcChannelsRaw message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
void
vdt_msgs__msg__RcChannelsRaw__fini(vdt_msgs__msg__RcChannelsRaw * msg);

/// Create msg/RcChannelsRaw message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * vdt_msgs__msg__RcChannelsRaw__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
vdt_msgs__msg__RcChannelsRaw *
vdt_msgs__msg__RcChannelsRaw__create(void);

/// Destroy msg/RcChannelsRaw message.
/**
 * It calls
 * vdt_msgs__msg__RcChannelsRaw__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
void
vdt_msgs__msg__RcChannelsRaw__destroy(vdt_msgs__msg__RcChannelsRaw * msg);

/// Check for msg/RcChannelsRaw message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
bool
vdt_msgs__msg__RcChannelsRaw__are_equal(const vdt_msgs__msg__RcChannelsRaw * lhs, const vdt_msgs__msg__RcChannelsRaw * rhs);

/// Copy a msg/RcChannelsRaw message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
bool
vdt_msgs__msg__RcChannelsRaw__copy(
  const vdt_msgs__msg__RcChannelsRaw * input,
  vdt_msgs__msg__RcChannelsRaw * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
const rosidl_type_hash_t *
vdt_msgs__msg__RcChannelsRaw__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
const rosidl_runtime_c__type_description__TypeDescription *
vdt_msgs__msg__RcChannelsRaw__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
const rosidl_runtime_c__type_description__TypeSource *
vdt_msgs__msg__RcChannelsRaw__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
vdt_msgs__msg__RcChannelsRaw__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/RcChannelsRaw messages.
/**
 * It allocates the memory for the number of elements and calls
 * vdt_msgs__msg__RcChannelsRaw__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
bool
vdt_msgs__msg__RcChannelsRaw__Sequence__init(vdt_msgs__msg__RcChannelsRaw__Sequence * array, size_t size);

/// Finalize array of msg/RcChannelsRaw messages.
/**
 * It calls
 * vdt_msgs__msg__RcChannelsRaw__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
void
vdt_msgs__msg__RcChannelsRaw__Sequence__fini(vdt_msgs__msg__RcChannelsRaw__Sequence * array);

/// Create array of msg/RcChannelsRaw messages.
/**
 * It allocates the memory for the array and calls
 * vdt_msgs__msg__RcChannelsRaw__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
vdt_msgs__msg__RcChannelsRaw__Sequence *
vdt_msgs__msg__RcChannelsRaw__Sequence__create(size_t size);

/// Destroy array of msg/RcChannelsRaw messages.
/**
 * It calls
 * vdt_msgs__msg__RcChannelsRaw__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
void
vdt_msgs__msg__RcChannelsRaw__Sequence__destroy(vdt_msgs__msg__RcChannelsRaw__Sequence * array);

/// Check for msg/RcChannelsRaw message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
bool
vdt_msgs__msg__RcChannelsRaw__Sequence__are_equal(const vdt_msgs__msg__RcChannelsRaw__Sequence * lhs, const vdt_msgs__msg__RcChannelsRaw__Sequence * rhs);

/// Copy an array of msg/RcChannelsRaw messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_vdt_msgs
bool
vdt_msgs__msg__RcChannelsRaw__Sequence__copy(
  const vdt_msgs__msg__RcChannelsRaw__Sequence * input,
  vdt_msgs__msg__RcChannelsRaw__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // VDT_MSGS__MSG__DETAIL__RC_CHANNELS_RAW__FUNCTIONS_H_
