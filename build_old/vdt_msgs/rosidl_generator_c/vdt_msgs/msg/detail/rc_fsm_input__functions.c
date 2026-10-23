// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vdt_msgs:msg/RcFsmInput.idl
// generated code does not contain a copyright notice
#include "vdt_msgs/msg/detail/rc_fsm_input__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
vdt_msgs__msg__RcFsmInput__init(vdt_msgs__msg__RcFsmInput * msg)
{
  if (!msg) {
    return false;
  }
  // land_switch
  // kill_switch
  return true;
}

void
vdt_msgs__msg__RcFsmInput__fini(vdt_msgs__msg__RcFsmInput * msg)
{
  if (!msg) {
    return;
  }
  // land_switch
  // kill_switch
}

bool
vdt_msgs__msg__RcFsmInput__are_equal(const vdt_msgs__msg__RcFsmInput * lhs, const vdt_msgs__msg__RcFsmInput * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // land_switch
  if (lhs->land_switch != rhs->land_switch) {
    return false;
  }
  // kill_switch
  if (lhs->kill_switch != rhs->kill_switch) {
    return false;
  }
  return true;
}

bool
vdt_msgs__msg__RcFsmInput__copy(
  const vdt_msgs__msg__RcFsmInput * input,
  vdt_msgs__msg__RcFsmInput * output)
{
  if (!input || !output) {
    return false;
  }
  // land_switch
  output->land_switch = input->land_switch;
  // kill_switch
  output->kill_switch = input->kill_switch;
  return true;
}

vdt_msgs__msg__RcFsmInput *
vdt_msgs__msg__RcFsmInput__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vdt_msgs__msg__RcFsmInput * msg = (vdt_msgs__msg__RcFsmInput *)allocator.allocate(sizeof(vdt_msgs__msg__RcFsmInput), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vdt_msgs__msg__RcFsmInput));
  bool success = vdt_msgs__msg__RcFsmInput__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vdt_msgs__msg__RcFsmInput__destroy(vdt_msgs__msg__RcFsmInput * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vdt_msgs__msg__RcFsmInput__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vdt_msgs__msg__RcFsmInput__Sequence__init(vdt_msgs__msg__RcFsmInput__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vdt_msgs__msg__RcFsmInput * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(vdt_msgs__msg__RcFsmInput)) {
      return false;
    }
    data = (vdt_msgs__msg__RcFsmInput *)allocator.zero_allocate(size, sizeof(vdt_msgs__msg__RcFsmInput), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vdt_msgs__msg__RcFsmInput__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vdt_msgs__msg__RcFsmInput__fini(&data[i - 1]);
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
vdt_msgs__msg__RcFsmInput__Sequence__fini(vdt_msgs__msg__RcFsmInput__Sequence * array)
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
      vdt_msgs__msg__RcFsmInput__fini(&array->data[i]);
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

vdt_msgs__msg__RcFsmInput__Sequence *
vdt_msgs__msg__RcFsmInput__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vdt_msgs__msg__RcFsmInput__Sequence * array = (vdt_msgs__msg__RcFsmInput__Sequence *)allocator.allocate(sizeof(vdt_msgs__msg__RcFsmInput__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vdt_msgs__msg__RcFsmInput__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vdt_msgs__msg__RcFsmInput__Sequence__destroy(vdt_msgs__msg__RcFsmInput__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vdt_msgs__msg__RcFsmInput__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vdt_msgs__msg__RcFsmInput__Sequence__are_equal(const vdt_msgs__msg__RcFsmInput__Sequence * lhs, const vdt_msgs__msg__RcFsmInput__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vdt_msgs__msg__RcFsmInput__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vdt_msgs__msg__RcFsmInput__Sequence__copy(
  const vdt_msgs__msg__RcFsmInput__Sequence * input,
  vdt_msgs__msg__RcFsmInput__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(vdt_msgs__msg__RcFsmInput)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(vdt_msgs__msg__RcFsmInput);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vdt_msgs__msg__RcFsmInput * data =
      (vdt_msgs__msg__RcFsmInput *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vdt_msgs__msg__RcFsmInput__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vdt_msgs__msg__RcFsmInput__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vdt_msgs__msg__RcFsmInput__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
