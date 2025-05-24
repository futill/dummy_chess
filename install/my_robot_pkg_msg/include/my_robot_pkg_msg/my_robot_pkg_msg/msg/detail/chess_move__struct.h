// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_robot_pkg_msg:msg/ChessMove.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_PKG_MSG__MSG__DETAIL__CHESS_MOVE__STRUCT_H_
#define MY_ROBOT_PKG_MSG__MSG__DETAIL__CHESS_MOVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'color'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ChessMove in the package my_robot_pkg_msg.
/**
  * msg/ChessMove.msg
 */
typedef struct my_robot_pkg_msg__msg__ChessMove
{
  /// 0-8
  uint8_t grid_index;
  /// "black" or "white"
  rosidl_runtime_c__String color;
} my_robot_pkg_msg__msg__ChessMove;

// Struct for a sequence of my_robot_pkg_msg__msg__ChessMove.
typedef struct my_robot_pkg_msg__msg__ChessMove__Sequence
{
  my_robot_pkg_msg__msg__ChessMove * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_pkg_msg__msg__ChessMove__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_ROBOT_PKG_MSG__MSG__DETAIL__CHESS_MOVE__STRUCT_H_
