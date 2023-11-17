cmake_minimum_required(VERSION 3.13)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

project(huron_examples)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/..)
find_package(huron REQUIRED)

include_directories(
    ${huron_INCLUDE_DIRS}
    ${sockcanpp_INCLUDE_DIRS}/..
    ${serial_INCLUDE_DIRS}/..)

add_subdirectory(test_can)
add_subdirectory(test_huron_canbus)
add_subdirectory(test_odrive_can)
add_subdirectory(test_serial)
