cmake_minimum_required(VERSION 3.5)
project(image_to_v4l2loopback)

# C++ Standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(stream
  src/image_converter.cpp
  src/stream.cpp
  src/video_device.cpp
)

ament_target_dependencies(stream
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
)

target_link_libraries(stream
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  stream
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Here you would define your tests
endif()

ament_package()
