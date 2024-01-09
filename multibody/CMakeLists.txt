set(dir "${CMAKE_CURRENT_SOURCE_DIR}")

list(APPEND SOURCE_FILES
  "${dir}/src/model.cc"
  "${dir}/src/joint_common.cc"
  "${dir}/src/frame.cc"
  "${dir}/src/logical_frame.cc"
  "${dir}/src/model_impl_default.cc")
# Check if using Pinocchio
if (DEFINED USE_PINOCCHIO)
  list(APPEND SOURCE_FILES
    "${dir}/src/pinocchio_model_impl.cc")
else()
  list(APPEND SOURCE_FILES
    "${dir}/src/no_pinocchio_model_impl.cc")
endif()

FILE(GLOB_RECURSE HEADER_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.cc
)

list(APPEND TEST_SOURCE_FILES
        "${dir}/test/test_model.cc"
        "${dir}/test/test_pinocchio_impl.cc")

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
