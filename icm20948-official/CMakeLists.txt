add_library(icm20948 STATIC)

target_sources(icm20948 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/ICM_20948.cpp)

target_include_directories(icm20948 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/util)

target_link_libraries(icm20948
    pico_stdlib
    hardware_i2c
)
