// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vdt_msgs:msg/InputSnapshot.idl
// generated code does not contain a copyright notice
#include "vdt_msgs/msg/detail/input_snapshot__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
vdt_msgs__msg__InputSnapshot__init(vdt_msgs__msg__InputSnapshot * msg)
{
  if (!msg) {
    return false;
  }
  // valid
  // marker_detected
  // align_error
  // altitude
  // delta_h
  // d_horiz
  // touchdown
  // planner_timeout
  // yaw_rate
  return true;
}

void
vdt_msgs__msg__InputSnapshot__fini(vdt_msgs__msg__InputSnapshot * msg)
{
  if (!msg) {
    return;
  }
  // valid
  // marker_detected
  // align_error
  // altitude
  // delta_h
  // d_horiz
  // touchdown
  // planner_timeout
  // yaw_rate
}

bool
vdt_msgs__msg__InputSnapshot__are_equal(const vdt_msgs__msg__InputSnapshot * lhs, const vdt_msgs__msg__InputSnapshot * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // valid
  if (lhs->valid != rhs->valid) {
    return false;
  }
  // marker_detected
  if (lhs->marker_detected != rhs->marker_detected) {
    return false;
  }
  // align_error
  if (lhs->align_error != rhs->align_error) {
    return false;
  }
  // altitude
  if (lhs->altitude != rhs->altitude) {
    return false;
  }
  // delta_h
  if (lhs->delta_h != rhs->delta_h) {
    return false;
  }
  // d_horiz
  if (lhs->d_horiz != rhs->d_horiz) {
    return false;
  }
  // touchdown
  if (lhs->touchdown != rhs->touchdown) {
    return false;
  }
  // planner_timeout
  if (lhs->planner_timeout != rhs->planner_timeout) {
    return false;
  }
  // yaw_rate
  if (lhs->yaw_rate != rhs->yaw_rate) {
    return false;
  }
  return true;
}

bool
vdt_msgs__msg__InputSnapshot__copy(
  const vdt_msgs__msg__InputSnapshot * input,
  vdt_msgs__msg__InputSnapshot * output)
{
  if (!input || !output) {
    return false;
  }
  // valid
  output->valid = input->valid;
  // marker_detected
  output->marker_detected = input->marker_detected;
  // align_error
  output->align_error = input->align_error;
  // altitude
  output->altitude = input->altitude;
  // delta_h
  output->delta_h = input->delta_h;
  // d_horiz
  output->d_horiz = input->d_horiz;
  // touchdown
  output->touchdown = input->touchdown;
  // planner_timeout
  output->planner_timeout = input->planner_timeout;
  // yaw_rate
  output->yaw_rate = input->yaw_rate;
  return true;
}

vdt_msgs__msg__InputSnapshot *
vdt_msgs__msg__InputSnapshot__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vdt_msgs__msg__InputSnapshot * msg = (vdt_msgs__msg__InputSnapshot *)allocator.allocate(sizeof(vdt_msgs__msg__InputSnapshot), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vdt_msgs__msg__InputSnapshot));
  bool success = vdt_msgs__msg__InputSnapshot__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vdt_msgs__msg__InputSnapshot__destroy(vdt_msgs__msg__InputSnapshot * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vdt_msgs__msg__InputSnapshot__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vdt_msgs__msg__InputSnapshot__Sequence__init(vdt_msgs__msg__InputSnapshot__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vdt_msgs__msg__InputSnapshot * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(vdt_msgs__msg__InputSnapshot)) {
      return false;
    }
    data = (vdt_msgs__msg__InputSnapshot *)allocator.zero_allocate(size, sizeof(vdt_msgs__msg__InputSnapshot), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vdt_msgs__msg__InputSnapshot__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vdt_msgs__msg__InputSnapshot__fini(&data[i - 1]);
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
vdt_msgs__msg__InputSnapshot__Sequence__fini(vdt_msgs__msg__InputSnapshot__Sequence * array)
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
      vdt_msgs__msg__InputSnapshot__fini(&array->data[i]);
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

vdt_msgs__msg__InputSnapshot__Sequence *
vdt_msgs__msg__InputSnapshot__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vdt_msgs__msg__InputSnapshot__Sequence * array = (vdt_msgs__msg__InputSnapshot__Sequence *)allocator.allocate(sizeof(vdt_msgs__msg__InputSnapshot__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vdt_msgs__msg__InputSnapshot__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vdt_msgs__msg__InputSnapshot__Sequence__destroy(vdt_msgs__msg__InputSnapshot__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vdt_msgs__msg__InputSnapshot__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vdt_msgs__msg__InputSnapshot__Sequence__are_equal(const vdt_msgs__msg__InputSnapshot__Sequence * lhs, const vdt_msgs__msg__InputSnapshot__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vdt_msgs__msg__InputSnapshot__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vdt_msgs__msg__InputSnapshot__Sequence__copy(
  const vdt_msgs__msg__InputSnapshot__Sequence * input,
  vdt_msgs__msg__InputSnapshot__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(vdt_msgs__msg__InputSnapshot)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(vdt_msgs__msg__InputSnapshot);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vdt_msgs__msg__InputSnapshot * data =
      (vdt_msgs__msg__InputSnapshot *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vdt_msgs__msg__InputSnapshot__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vdt_msgs__msg__InputSnapshot__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vdt_msgs__msg__InputSnapshot__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
