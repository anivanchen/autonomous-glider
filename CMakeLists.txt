cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(autonomous-glider C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

add_subdirectory("icm20948-official")

add_executable(autonomous-glider src/main.cpp)

target_link_libraries(autonomous-glider PRIVATE pico_stdlib hardware_i2c icm20948)

pico_enable_stdio_usb(autonomous-glider 1)
pico_enable_stdio_uart(autonomous-glider 0)

pico_add_extra_outputs(autonomous-glider)
