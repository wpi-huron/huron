set(dir "${CMAKE_CURRENT_SOURCE_DIR}")

list(APPEND SOURCE_FILES
  "${dir}/src/model.cc"
  "${dir}/src/model_impl_base.cc")
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
