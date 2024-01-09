cmake_minimum_required(VERSION 3.8)
project(huron_ros2)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


find_package(huron REQUIRED)

add_library(${PROJECT_NAME}
  SHARED
  src/huron.cc
  src/huron_node.cc
  src/joint_state_provider.cc
  src/force_torque_sensor.cc
  src/joint_group_controller.cc
)

# target_link_libraries(${PROJECT_NAME} PUBLIC ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES} huron::huron)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp sensor_msgs geometry_msgs nav_msgs)
target_link_libraries(${PROJECT_NAME} PUBLIC huron::huron)

target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(huron)

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
