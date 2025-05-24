// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_robot_pkg_msg:msg/ChessMove.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_PKG_MSG__MSG__DETAIL__CHESS_MOVE__TRAITS_HPP_
#define MY_ROBOT_PKG_MSG__MSG__DETAIL__CHESS_MOVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "my_robot_pkg_msg/msg/detail/chess_move__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace my_robot_pkg_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const ChessMove & msg,
  std::ostream & out)
{
  out << "{";
  // member: grid_index
  {
    out << "grid_index: ";
    rosidl_generator_traits::value_to_yaml(msg.grid_index, out);
    out << ", ";
  }

  // member: color
  {
    out << "color: ";
    rosidl_generator_traits::value_to_yaml(msg.color, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChessMove & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: grid_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "grid_index: ";
    rosidl_generator_traits::value_to_yaml(msg.grid_index, out);
    out << "\n";
  }

  // member: color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color: ";
    rosidl_generator_traits::value_to_yaml(msg.color, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChessMove & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace my_robot_pkg_msg

namespace rosidl_generator_traits
{

[[deprecated("use my_robot_pkg_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_robot_pkg_msg::msg::ChessMove & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_robot_pkg_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_robot_pkg_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const my_robot_pkg_msg::msg::ChessMove & msg)
{
  return my_robot_pkg_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<my_robot_pkg_msg::msg::ChessMove>()
{
  return "my_robot_pkg_msg::msg::ChessMove";
}

template<>
inline const char * name<my_robot_pkg_msg::msg::ChessMove>()
{
  return "my_robot_pkg_msg/msg/ChessMove";
}

template<>
struct has_fixed_size<my_robot_pkg_msg::msg::ChessMove>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<my_robot_pkg_msg::msg::ChessMove>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<my_robot_pkg_msg::msg::ChessMove>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MY_ROBOT_PKG_MSG__MSG__DETAIL__CHESS_MOVE__TRAITS_HPP_
