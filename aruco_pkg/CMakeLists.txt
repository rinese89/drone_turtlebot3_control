cmake_minimum_required(VERSION 3.8)
project(aruco_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(aruco_pub_node
src/aruco_pub_node.cpp)

ament_target_dependencies(
aruco_pub_node
rclcpp
geometry_msgs
tf2_geometry_msgs
tf2_ros
tf2
OpenCV
)

install(
  TARGETS aruco_pub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
