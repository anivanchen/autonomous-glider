add_library(icm20948 STATIC)

target_sources(icm20948 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/ICM_20948.cpp ${CMAKE_CURRENT_LIST_DIR}/ICM_20948_C.c)

target_link_libraries(icm20948
    pico_stdlib
    hardware_i2c
)
