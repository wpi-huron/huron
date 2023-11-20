add_executable(test_fsr_array_serial main.cc)

target_link_libraries(test_fsr_array_serial
    PUBLIC
    huron::huron
)
target_include_directories(test_fsr_array_serial
    PUBLIC
    ${serial_INCLUDE_DIRS}/..)
