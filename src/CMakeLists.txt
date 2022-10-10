add_executable(pico-rtos
        main.c
)

target_link_libraries(pico-rtos pico_stdlib freertos)
pico_add_extra_outputs(pico-rtos)