add_executable(pico-rtos
        main.c
)

target_link_libraries(pico-rtos pico_stdlib freertos icm20948 hardware_i2c)

pico_enable_stdio_usb(pico-rtos 1)
pico_enable_stdio_uart(pico-rtos 0)

pico_add_extra_outputs(pico-rtos)