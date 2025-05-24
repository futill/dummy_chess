// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_robot_pkg_msg:msg/ChessMove.idl
// generated code does not contain a copyright notice
#include "my_robot_pkg_msg/msg/detail/chess_move__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `color`
#include "rosidl_runtime_c/string_functions.h"

bool
my_robot_pkg_msg__msg__ChessMove__init(my_robot_pkg_msg__msg__ChessMove * msg)
{
  if (!msg) {
    return false;
  }
  // grid_index
  // color
  if (!rosidl_runtime_c__String__init(&msg->color)) {
    my_robot_pkg_msg__msg__ChessMove__fini(msg);
    return false;
  }
  return true;
}

void
my_robot_pkg_msg__msg__ChessMove__fini(my_robot_pkg_msg__msg__ChessMove * msg)
{
  if (!msg) {
    return;
  }
  // grid_index
  // color
  rosidl_runtime_c__String__fini(&msg->color);
}

bool
my_robot_pkg_msg__msg__ChessMove__are_equal(const my_robot_pkg_msg__msg__ChessMove * lhs, const my_robot_pkg_msg__msg__ChessMove * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // grid_index
  if (lhs->grid_index != rhs->grid_index) {
    return false;
  }
  // color
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->color), &(rhs->color)))
  {
    return false;
  }
  return true;
}

bool
my_robot_pkg_msg__msg__ChessMove__copy(
  const my_robot_pkg_msg__msg__ChessMove * input,
  my_robot_pkg_msg__msg__ChessMove * output)
{
  if (!input || !output) {
    return false;
  }
  // grid_index
  output->grid_index = input->grid_index;
  // color
  if (!rosidl_runtime_c__String__copy(
      &(input->color), &(output->color)))
  {
    return false;
  }
  return true;
}

my_robot_pkg_msg__msg__ChessMove *
my_robot_pkg_msg__msg__ChessMove__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_robot_pkg_msg__msg__ChessMove * msg = (my_robot_pkg_msg__msg__ChessMove *)allocator.allocate(sizeof(my_robot_pkg_msg__msg__ChessMove), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_robot_pkg_msg__msg__ChessMove));
  bool success = my_robot_pkg_msg__msg__ChessMove__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_robot_pkg_msg__msg__ChessMove__destroy(my_robot_pkg_msg__msg__ChessMove * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_robot_pkg_msg__msg__ChessMove__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_robot_pkg_msg__msg__ChessMove__Sequence__init(my_robot_pkg_msg__msg__ChessMove__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_robot_pkg_msg__msg__ChessMove * data = NULL;

  if (size) {
    data = (my_robot_pkg_msg__msg__ChessMove *)allocator.zero_allocate(size, sizeof(my_robot_pkg_msg__msg__ChessMove), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_robot_pkg_msg__msg__ChessMove__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_robot_pkg_msg__msg__ChessMove__fini(&data[i - 1]);
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
my_robot_pkg_msg__msg__ChessMove__Sequence__fini(my_robot_pkg_msg__msg__ChessMove__Sequence * array)
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
      my_robot_pkg_msg__msg__ChessMove__fini(&array->data[i]);
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

my_robot_pkg_msg__msg__ChessMove__Sequence *
my_robot_pkg_msg__msg__ChessMove__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_robot_pkg_msg__msg__ChessMove__Sequence * array = (my_robot_pkg_msg__msg__ChessMove__Sequence *)allocator.allocate(sizeof(my_robot_pkg_msg__msg__ChessMove__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_robot_pkg_msg__msg__ChessMove__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_robot_pkg_msg__msg__ChessMove__Sequence__destroy(my_robot_pkg_msg__msg__ChessMove__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_robot_pkg_msg__msg__ChessMove__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_robot_pkg_msg__msg__ChessMove__Sequence__are_equal(const my_robot_pkg_msg__msg__ChessMove__Sequence * lhs, const my_robot_pkg_msg__msg__ChessMove__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_robot_pkg_msg__msg__ChessMove__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_robot_pkg_msg__msg__ChessMove__Sequence__copy(
  const my_robot_pkg_msg__msg__ChessMove__Sequence * input,
  my_robot_pkg_msg__msg__ChessMove__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_robot_pkg_msg__msg__ChessMove);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    my_robot_pkg_msg__msg__ChessMove * data =
      (my_robot_pkg_msg__msg__ChessMove *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_robot_pkg_msg__msg__ChessMove__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          my_robot_pkg_msg__msg__ChessMove__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!my_robot_pkg_msg__msg__ChessMove__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
