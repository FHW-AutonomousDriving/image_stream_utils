cmake_minimum_required(VERSION 3.5)
project(image_stream_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)

# pipewrite node
add_executable(pipewrite src/pipewrite.cpp)
ament_target_dependencies(
  pipewrite
  "rclcpp"
  "image_transport"
)
install(TARGETS pipewrite
  DESTINATION lib/${PROJECT_NAME})

ament_package()
