cmake_minimum_required(VERSION 3.8)
project(examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_PREFIX_PATH
        "/usr/local/"
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(huron_ros2 REQUIRED)
find_package(huron REQUIRED)

add_executable(test_pub_sub src/test_pub_sub.cc)
add_executable(test_push_recovery src/test_push_recovery.cc)
ament_target_dependencies(test_pub_sub rclcpp sensor_msgs nav_msgs huron_ros2)
ament_target_dependencies(test_push_recovery huron rclcpp sensor_msgs nav_msgs huron_ros2)


install(TARGETS
        test_pub_sub
        test_push_recovery
        DESTINATION lib/${PROJECT_NAME})


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

ament_package()
