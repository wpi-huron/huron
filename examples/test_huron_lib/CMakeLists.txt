add_executable(test_huron_lib main.cc)

target_link_libraries(test_huron_lib
    PUBLIC
    huron::huron
)

