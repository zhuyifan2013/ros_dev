cmake_minimum_required(VERSION 3.8)
project(tarkbot_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED COMPONENTS 
  system
  thread
)


set(msg_files
   "srv/LightSet.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

include_directories(
  include
  include/${PROJECT_NAME}
  ${Boost_INCLUDE_DIRS}
)

add_executable(tarkbot_robot_pipi src/tarkbot_robot.cpp)
ament_target_dependencies(tarkbot_robot_pipi
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(tarkbot_robot_pipi "${cpp_typesupport_target}")
target_link_libraries(tarkbot_robot_pipi
  ${Boost_LIBRARIES}
)

install(TARGETS
  tarkbot_robot_pipi
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  rosidl_default_runtime
)

ament_export_include_directories(include)

ament_package()