cmake_minimum_required(VERSION 3.8)
project(bluerov2_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(thruster_allocator src/thruster_allocator.cpp)
ament_target_dependencies(thruster_allocator
  geometry_msgs
  rclcpp
  std_msgs
)

install(TARGETS
  thruster_allocator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
