cmake_minimum_required(VERSION 3.5)

project(ros2socketcan_bridge)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

#TARGET_LINK_LIBRARIES(ros_iblock mylib)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(rclc REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
#find_package(iblock_interfaces REQUIRED)
find_package(can_msgs REQUIRED)
#find_package(rmw_implementation REQUIRED)

include_directories(include)

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin ) 
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib ) 
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )


#add_executable(ros2can src/ros2can.cpp)
#add_executable(can2ros src/can2ros.cpp)
add_executable(ros2can_bridge src/ros2socketcan.cpp)

#TARGET_LINK_LIBRARIES(ros2can dl -lboost_system)# -pthread  -lcrypt -lm -lpthread)
#TARGET_LINK_LIBRARIES(can2ros dl -lboost_system)# -pthread  -lcrypt -lm -lpthread)
TARGET_LINK_LIBRARIES(ros2can_bridge dl -lboost_system)# -pthread  -lcrypt -lm -lpthread)

#ament_target_dependencies(ros2can rclcpp std_msgs can_msgs)
#ament_target_dependencies(can2ros rclcpp std_msgs can_msgs)
ament_target_dependencies(ros2can_bridge rclcpp std_msgs can_msgs)
install(TARGETS #ros2can
                #can2ros
                ros2can_bridge
        DESTINATION bin)
        
ament_package()
