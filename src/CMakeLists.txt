add_executable(pico-rtos
        main.c
)

target_link_libraries(pico-rtos pico_stdlib freertos)

pico_enable_stdio_usb(pico-rtos 1)

pico_enable_stdio_uart(pico-rtos 0)

pico_add_extra_outputs(pico-rtos)