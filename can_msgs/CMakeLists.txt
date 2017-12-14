cmake_minimum_required(VERSION 3.5)

project(can_msgs)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -Wall -Wextra")
endif()

find_package(ament_cmake               REQUIRED)
find_package(builtin_interfaces        REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs             REQUIRED)
find_package(std_msgs                  REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Frame.msg"
  DEPENDENCIES builtin_interfaces  geometry_msgs std_msgs
)

ament_package()
