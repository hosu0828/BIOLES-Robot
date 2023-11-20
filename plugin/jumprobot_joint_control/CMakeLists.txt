cmake_minimum_required(VERSION 3.5)
project(jumprobot_joint_control)

# Find Gazebo
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${rclcpp_INCLUDE_DIRS})


link_directories(${GAZEBO_LIBRARY_DIRS})
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set (CMAKE_CXX_STANDARD 11)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(jumprobot_joint_control SHARED jumprobot_joint_control.cc)
target_link_libraries(jumprobot_joint_control ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})