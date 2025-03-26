cmake_minimum_required(VERSION 3.8)

project(m1ct_d2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(rosidl_default_generators REQUIRED)

set(SDK_PATH "./src/")

FILE(GLOB SDK_SRC 
  "${SDK_PATH}*.cpp"
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include/m1ct_d2
)

add_executable(m1ct_d2 ${SDK_SRC})

ament_target_dependencies(m1ct_d2
  "rclcpp"
  "sensor_msgs"
  "visualization_msgs"
  "geometry_msgs"
)

install(TARGETS
  m1ct_d2
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
