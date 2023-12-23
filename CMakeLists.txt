cmake_minimum_required(VERSION 3.13)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include(pico_sdk_import.cmake)
project(pico_serprog)

pico_sdk_init()

add_executable(pico_serprog)
pico_generate_pio_header(pico_i2s_mic ${CMAKE_CURRENT_LIST_DIR}/mic_i2s.pio)
target_sources(pico_serprog PRIVATE main.c usb_descriptors.c pio_spi.c)
target_link_libraries(pico_serprog PRIVATE pico_stdlib hardware_spi hardware_pio tinyusb_device)
pico_add_extra_outputs(pico_serprog)
