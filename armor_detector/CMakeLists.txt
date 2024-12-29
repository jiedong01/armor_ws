cmake_minimum_required(VERSION 3.10)
project(armor_detector)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

add_executable(armor_detector
  src/armor_detector_node.cpp
  tasks/detector.cpp
)

ament_target_dependencies(armor_detector rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS
  armor_detector
  DESTINATION lib/${PROJECT_NAME})

ament_package()