set(dir "${CMAKE_CURRENT_SOURCE_DIR}")

list(APPEND SOURCE_FILES
  "${dir}/src/configuration.cc"
  "${dir}/src/moving_group.cc"
  "${dir}/src/joint.cc"
  "${dir}/src/sensor.cc"
  "${dir}/src/sensor_with_frame.cc"
  "${dir}/src/robot.cc"
  "${dir}/src/legged_robot.cc")
FILE(GLOB_RECURSE HEADER_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
)

list(APPEND TEST_SOURCE_FILES
        "${dir}/test/test_legged_zmp.cc")

set(INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include/")

target_sources(huron
    PRIVATE
    ${SOURCE_FILES}
    # PUBLIC
    # ${HEADER_FILES}
)
target_include_directories(huron
    PUBLIC
    $<BUILD_INTERFACE:${INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include> 
)

target_sources(huron_tests
  PRIVATE
  ${TEST_SOURCE_FILES}
)
target_include_directories(huron_tests
        PUBLIC
        $<BUILD_INTERFACE:${INCLUDE_DIRS}>
        $<INSTALL_INTERFACE:include>
)

install(DIRECTORY ${INCLUDE_DIRS} DESTINATION include)
