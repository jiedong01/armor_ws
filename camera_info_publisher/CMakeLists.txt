cmake_minimum_required(VERSION 3.8)
project(camera_info_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED) # 添加cv_bridge依赖
find_package(OpenCV REQUIRED) # 添加OpenCV依赖
include_directories( ${OpenCV_INCLUDE_DIRS} )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(camera_info_publisher src/camera_info_publisher.cpp)
ament_target_dependencies(camera_info_publisher
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV) # 添加cv_bridge和OpenCV到目标依赖

target_link_libraries(camera_info_publisher ${OpenCV_LIBS} )

install(TARGETS
  camera_info_publisher
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()