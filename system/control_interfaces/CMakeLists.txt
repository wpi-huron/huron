set(dir "${CMAKE_CURRENT_SOURCE_DIR}")

list(APPEND SOURCE_FILES
  "${dir}/src/configuration.cc"
  "${dir}/src/moving_group.cc"
  "${dir}/src/joint.cc"
  "${dir}/src/sensor.cc"
  "${dir}/src/sensor_with_frame.cc"
  "${dir}/src/robot.cc")
FILE(GLOB_RECURSE HEADER_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
)

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

install(DIRECTORY ${INCLUDE_DIRS} DESTINATION include)
