cmake_minimum_required(VERSION 3.5)
project(video_publisher_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(video_publisher_cpp_node src/video_publisher_cpp.cpp)

ament_target_dependencies(video_publisher_cpp_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(video_publisher_cpp_node
  ${OpenCV_LIBS}
)

install(TARGETS
  video_publisher_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
