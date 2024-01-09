set(dir "${CMAKE_CURRENT_SOURCE_DIR}")

list(APPEND SOURCE_FILES
  "${dir}/src/force_sensing_resistor.cc"
  "${dir}/src/force_sensing_resistor_array.cc"
  "${dir}/src/force_sensing_resistor_array_serial.cc"
  "${dir}/src/force_torque_sensor.cc")
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
