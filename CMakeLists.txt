cmake_minimum_required(VERSION 3.12)

# initialize pico-sdk from submodule
# note: this must happen before project()
include(pico-sdk/pico_sdk_init.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(freertos)
add_subdirectory(icm20948)
include_directories(icm20948)
add_subdirectory(src)

