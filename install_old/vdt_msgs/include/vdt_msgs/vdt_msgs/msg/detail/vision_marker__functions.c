// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vdt_msgs:msg/VisionMarker.idl
// generated code does not contain a copyright notice
#include "vdt_msgs/msg/detail/vision_marker__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
vdt_msgs__msg__VisionMarker__init(vdt_msgs__msg__VisionMarker * msg)
{
  if (!msg) {
    return false;
  }
  // marker_visible
  // pixel_align_error
  return true;
}

void
vdt_msgs__msg__VisionMarker__fini(vdt_msgs__msg__VisionMarker * msg)
{
  if (!msg) {
    return;
  }
  // marker_visible
  // pixel_align_error
}

bool
vdt_msgs__msg__VisionMarker__are_equal(const vdt_msgs__msg__VisionMarker * lhs, const vdt_msgs__msg__VisionMarker * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // marker_visible
  if (lhs->marker_visible != rhs->marker_visible) {
    return false;
  }
  // pixel_align_error
  if (lhs->pixel_align_error != rhs->pixel_align_error) {
    return false;
  }
  return true;
}

bool
vdt_msgs__msg__VisionMarker__copy(
  const vdt_msgs__msg__VisionMarker * input,
  vdt_msgs__msg__VisionMarker * output)
{
  if (!input || !output) {
    return false;
  }
  // marker_visible
  output->marker_visible = input->marker_visible;
  // pixel_align_error
  output->pixel_align_error = input->pixel_align_error;
  return true;
}

vdt_msgs__msg__VisionMarker *
vdt_msgs__msg__VisionMarker__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vdt_msgs__msg__VisionMarker * msg = (vdt_msgs__msg__VisionMarker *)allocator.allocate(sizeof(vdt_msgs__msg__VisionMarker), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vdt_msgs__msg__VisionMarker));
  bool success = vdt_msgs__msg__VisionMarker__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vdt_msgs__msg__VisionMarker__destroy(vdt_msgs__msg__VisionMarker * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vdt_msgs__msg__VisionMarker__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vdt_msgs__msg__VisionMarker__Sequence__init(vdt_msgs__msg__VisionMarker__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vdt_msgs__msg__VisionMarker * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(vdt_msgs__msg__VisionMarker)) {
      return false;
    }
    data = (vdt_msgs__msg__VisionMarker *)allocator.zero_allocate(size, sizeof(vdt_msgs__msg__VisionMarker), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vdt_msgs__msg__VisionMarker__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vdt_msgs__msg__VisionMarker__fini(&data[i - 1]);
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
vdt_msgs__msg__VisionMarker__Sequence__fini(vdt_msgs__msg__VisionMarker__Sequence * array)
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
      vdt_msgs__msg__VisionMarker__fini(&array->data[i]);
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

vdt_msgs__msg__VisionMarker__Sequence *
vdt_msgs__msg__VisionMarker__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vdt_msgs__msg__VisionMarker__Sequence * array = (vdt_msgs__msg__VisionMarker__Sequence *)allocator.allocate(sizeof(vdt_msgs__msg__VisionMarker__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vdt_msgs__msg__VisionMarker__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vdt_msgs__msg__VisionMarker__Sequence__destroy(vdt_msgs__msg__VisionMarker__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vdt_msgs__msg__VisionMarker__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vdt_msgs__msg__VisionMarker__Sequence__are_equal(const vdt_msgs__msg__VisionMarker__Sequence * lhs, const vdt_msgs__msg__VisionMarker__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vdt_msgs__msg__VisionMarker__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vdt_msgs__msg__VisionMarker__Sequence__copy(
  const vdt_msgs__msg__VisionMarker__Sequence * input,
  vdt_msgs__msg__VisionMarker__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(vdt_msgs__msg__VisionMarker)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(vdt_msgs__msg__VisionMarker);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vdt_msgs__msg__VisionMarker * data =
      (vdt_msgs__msg__VisionMarker *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vdt_msgs__msg__VisionMarker__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vdt_msgs__msg__VisionMarker__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vdt_msgs__msg__VisionMarker__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
