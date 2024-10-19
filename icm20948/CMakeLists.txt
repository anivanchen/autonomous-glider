add_library(icm20948 STATIC)

target_sources(icm20948 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/icm20948.c)

target_link_libraries(icm20948
    pico_stdlib
    hardware_i2c
)
