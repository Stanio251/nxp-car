cmake_minimum_required(VERSION 3.8)
project(angle_pid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(angle_pid_node src/angle_pid.cpp)  
ament_target_dependencies(angle_pid_node
  rclcpp
  std_msgs
  geometry_msgs 
)

install(TARGETS angle_pid_node DESTINATION lib/${PROJECT_NAME}) 

ament_package()
