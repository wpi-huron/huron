add_executable(hello_world hello_world.cc)

target_link_libraries(hello_world
    PUBLIC
    huron::huron
)
target_include_directories(hello_world
    PUBLIC
    ${serial_INCLUDE_DIRS}/..)
