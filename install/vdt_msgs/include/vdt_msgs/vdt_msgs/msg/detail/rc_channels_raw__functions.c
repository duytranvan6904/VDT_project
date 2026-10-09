// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vdt_msgs:msg/RcChannelsRaw.idl
// generated code does not contain a copyright notice
#include "vdt_msgs/msg/detail/rc_channels_raw__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
vdt_msgs__msg__RcChannelsRaw__init(vdt_msgs__msg__RcChannelsRaw * msg)
{
  if (!msg) {
    return false;
  }
  // ch
  // valid
  // failsafe
  return true;
}

void
vdt_msgs__msg__RcChannelsRaw__fini(vdt_msgs__msg__RcChannelsRaw * msg)
{
  if (!msg) {
    return;
  }
  // ch
  // valid
  // failsafe
}

bool
vdt_msgs__msg__RcChannelsRaw__are_equal(const vdt_msgs__msg__RcChannelsRaw * lhs, const vdt_msgs__msg__RcChannelsRaw * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ch
  for (size_t i = 0; i < 16; ++i) {
    if (lhs->ch[i] != rhs->ch[i]) {
      return false;
    }
  }
  // valid
  if (lhs->valid != rhs->valid) {
    return false;
  }
  // failsafe
  if (lhs->failsafe != rhs->failsafe) {
    return false;
  }
  return true;
}

bool
vdt_msgs__msg__RcChannelsRaw__copy(
  const vdt_msgs__msg__RcChannelsRaw * input,
  vdt_msgs__msg__RcChannelsRaw * output)
{
  if (!input || !output) {
    return false;
  }
  // ch
  for (size_t i = 0; i < 16; ++i) {
    output->ch[i] = input->ch[i];
  }
  // valid
  output->valid = input->valid;
  // failsafe
  output->failsafe = input->failsafe;
  return true;
}

vdt_msgs__msg__RcChannelsRaw *
vdt_msgs__msg__RcChannelsRaw__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vdt_msgs__msg__RcChannelsRaw * msg = (vdt_msgs__msg__RcChannelsRaw *)allocator.allocate(sizeof(vdt_msgs__msg__RcChannelsRaw), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vdt_msgs__msg__RcChannelsRaw));
  bool success = vdt_msgs__msg__RcChannelsRaw__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vdt_msgs__msg__RcChannelsRaw__destroy(vdt_msgs__msg__RcChannelsRaw * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vdt_msgs__msg__RcChannelsRaw__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vdt_msgs__msg__RcChannelsRaw__Sequence__init(vdt_msgs__msg__RcChannelsRaw__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vdt_msgs__msg__RcChannelsRaw * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(vdt_msgs__msg__RcChannelsRaw)) {
      return false;
    }
    data = (vdt_msgs__msg__RcChannelsRaw *)allocator.zero_allocate(size, sizeof(vdt_msgs__msg__RcChannelsRaw), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vdt_msgs__msg__RcChannelsRaw__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vdt_msgs__msg__RcChannelsRaw__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
vdt_msgs__msg__RcChannelsRaw__Sequence__fini(vdt_msgs__msg__RcChannelsRaw__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      vdt_msgs__msg__RcChannelsRaw__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

vdt_msgs__msg__RcChannelsRaw__Sequence *
vdt_msgs__msg__RcChannelsRaw__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vdt_msgs__msg__RcChannelsRaw__Sequence * array = (vdt_msgs__msg__RcChannelsRaw__Sequence *)allocator.allocate(sizeof(vdt_msgs__msg__RcChannelsRaw__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vdt_msgs__msg__RcChannelsRaw__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vdt_msgs__msg__RcChannelsRaw__Sequence__destroy(vdt_msgs__msg__RcChannelsRaw__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vdt_msgs__msg__RcChannelsRaw__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vdt_msgs__msg__RcChannelsRaw__Sequence__are_equal(const vdt_msgs__msg__RcChannelsRaw__Sequence * lhs, const vdt_msgs__msg__RcChannelsRaw__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vdt_msgs__msg__RcChannelsRaw__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vdt_msgs__msg__RcChannelsRaw__Sequence__copy(
  const vdt_msgs__msg__RcChannelsRaw__Sequence * input,
  vdt_msgs__msg__RcChannelsRaw__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(vdt_msgs__msg__RcChannelsRaw)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(vdt_msgs__msg__RcChannelsRaw);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vdt_msgs__msg__RcChannelsRaw * data =
      (vdt_msgs__msg__RcChannelsRaw *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vdt_msgs__msg__RcChannelsRaw__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vdt_msgs__msg__RcChannelsRaw__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vdt_msgs__msg__RcChannelsRaw__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
