cmake_minimum_required(VERSION 3.8)
project(drone_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#------ find dependencies --------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(drone_actions REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) 
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED) 

#-------- Headers files ----------------

include_directories(
  include
)

#---------- Libraries ------------------

add_library(
  sockets_lib
  "src/sockets/ClientNetwork.cpp"
  "src/sockets/NetworkServices.cpp"
)

#---- Executables & Dependencies -----

add_executable(drone_server_node
"src/drone_server_node.cpp")

ament_target_dependencies(
  drone_server_node
  "rclcpp"
  "rclcpp_action"
  "drone_actions"
  "drone_msgs"
  "geometry_msgs"
  "tf2_geometry_msgs"

)

#---- Executables & Dependencies -----

add_executable(
  drone_client_lc_node
"src/drone_client_lc_node.cpp")

ament_target_dependencies(
  drone_client_lc_node
  "rclcpp"
  "rclcpp_action"
  "drone_actions"
  "drone_msgs"
  "geometry_msgs"
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  "tf2_geometry_msgs"
)

#--------- Link libraries ---------------

target_link_libraries(
  drone_server_node
  sockets_lib
)

#--------- Move to install directory ----

install(TARGETS
drone_server_node
drone_client_lc_node
DESTINATION lib/${PROJECT_NAME}
)
ament_package()
