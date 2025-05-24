// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_robot_pkg_msg:msg/ChessMove.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_PKG_MSG__MSG__DETAIL__CHESS_MOVE__STRUCT_HPP_
#define MY_ROBOT_PKG_MSG__MSG__DETAIL__CHESS_MOVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__my_robot_pkg_msg__msg__ChessMove __attribute__((deprecated))
#else
# define DEPRECATED__my_robot_pkg_msg__msg__ChessMove __declspec(deprecated)
#endif

namespace my_robot_pkg_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ChessMove_
{
  using Type = ChessMove_<ContainerAllocator>;

  explicit ChessMove_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->grid_index = 0;
      this->color = "";
    }
  }

  explicit ChessMove_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : color(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->grid_index = 0;
      this->color = "";
    }
  }

  // field types and members
  using _grid_index_type =
    uint8_t;
  _grid_index_type grid_index;
  using _color_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _color_type color;

  // setters for named parameter idiom
  Type & set__grid_index(
    const uint8_t & _arg)
  {
    this->grid_index = _arg;
    return *this;
  }
  Type & set__color(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->color = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_robot_pkg_msg::msg::ChessMove_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_robot_pkg_msg::msg::ChessMove_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_robot_pkg_msg::msg::ChessMove_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_robot_pkg_msg::msg::ChessMove_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_robot_pkg_msg::msg::ChessMove_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_robot_pkg_msg::msg::ChessMove_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_robot_pkg_msg::msg::ChessMove_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_robot_pkg_msg::msg::ChessMove_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_robot_pkg_msg::msg::ChessMove_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_robot_pkg_msg::msg::ChessMove_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_robot_pkg_msg__msg__ChessMove
    std::shared_ptr<my_robot_pkg_msg::msg::ChessMove_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_robot_pkg_msg__msg__ChessMove
    std::shared_ptr<my_robot_pkg_msg::msg::ChessMove_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ChessMove_ & other) const
  {
    if (this->grid_index != other.grid_index) {
      return false;
    }
    if (this->color != other.color) {
      return false;
    }
    return true;
  }
  bool operator!=(const ChessMove_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ChessMove_

// alias to use template instance with default allocator
using ChessMove =
  my_robot_pkg_msg::msg::ChessMove_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace my_robot_pkg_msg

#endif  // MY_ROBOT_PKG_MSG__MSG__DETAIL__CHESS_MOVE__STRUCT_HPP_
