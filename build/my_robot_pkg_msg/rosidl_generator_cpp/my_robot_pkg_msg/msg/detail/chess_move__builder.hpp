// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_robot_pkg_msg:msg/ChessMove.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_PKG_MSG__MSG__DETAIL__CHESS_MOVE__BUILDER_HPP_
#define MY_ROBOT_PKG_MSG__MSG__DETAIL__CHESS_MOVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_robot_pkg_msg/msg/detail/chess_move__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_robot_pkg_msg
{

namespace msg
{

namespace builder
{

class Init_ChessMove_color
{
public:
  explicit Init_ChessMove_color(::my_robot_pkg_msg::msg::ChessMove & msg)
  : msg_(msg)
  {}
  ::my_robot_pkg_msg::msg::ChessMove color(::my_robot_pkg_msg::msg::ChessMove::_color_type arg)
  {
    msg_.color = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_pkg_msg::msg::ChessMove msg_;
};

class Init_ChessMove_grid_index
{
public:
  Init_ChessMove_grid_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChessMove_color grid_index(::my_robot_pkg_msg::msg::ChessMove::_grid_index_type arg)
  {
    msg_.grid_index = std::move(arg);
    return Init_ChessMove_color(msg_);
  }

private:
  ::my_robot_pkg_msg::msg::ChessMove msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_pkg_msg::msg::ChessMove>()
{
  return my_robot_pkg_msg::msg::builder::Init_ChessMove_grid_index();
}

}  // namespace my_robot_pkg_msg

#endif  // MY_ROBOT_PKG_MSG__MSG__DETAIL__CHESS_MOVE__BUILDER_HPP_
