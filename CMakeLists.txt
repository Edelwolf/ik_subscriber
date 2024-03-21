cmake_minimum_required(VERSION 3.8)
project(ik_subscriber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(interfaces_for_ik_service REQUIRED)
find_package(ik_solver REQUIRED)
find_package(ik_interfaces REQUIRED)

# Add include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${ik_interfaces_INCLUDE_DIRS}
)

# Add executable
add_executable(ik_subscriber src/ik_subscriber.cpp)
# Add dependencies to executable
ament_target_dependencies(ik_subscriber
  rclcpp
  geometry_msgs
  ik_interfaces
)
# Add dependencies to executable
ament_target_dependencies(
ik_subscriber
rclcpp
geometry_msgs
ik_solver)


# Install executable
install(TARGETS
  ik_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, if any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Export package configuration
ament_export_dependencies(
rclcpp
ik_solver)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
