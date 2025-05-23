cmake_minimum_required(VERSION 3.5)
project(my_robot_pkg_msg)

# 必需依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 可选依赖（根据 ROS 发行版判断）
if("$ENV{ROS_DISTRO}" STREQUAL "iron" OR "$ENV{ROS_DISTRO}" STREQUAL "rolling")
    find_package(service_msgs REQUIRED)
    set(SERVICE_MSGS_DEPS service_msgs)
endif()

# 消息所需依赖
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChessMove.msg"
  DEPENDENCIES std_msgs geometry_msgs ${SERVICE_MSGS_DEPS}
)

# 安装、导出依赖
ament_export_dependencies(rosidl_default_runtime)
ament_package()
