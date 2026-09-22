cmake_minimum_required(VERSION 3.16)

project(rccar_firmware C ASM)

option(RCCAR_BUILD_HOST_TESTS "Build host-side parser tests" ON)
option(RCCAR_BUILD_FIRMWARE "Build STM32 firmware with arm-none-eabi-gcc" OFF)
option(RCCAR_ENABLE_ESC_FE32_REPLAY "Run FE32 replay tests against the full real capture files" ON)
set(RCCAR_ESC_FE32_REPLAY_ROOT "${CMAKE_SOURCE_DIR}/../资料/实测记录"
    CACHE PATH "Directory containing the full FE32 real capture recordings")

function(rccar_enable_strict_c_warnings target_name)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target_name} PRIVATE -Wall -Wextra -Werror)
    endif()
endfunction()

add_library(esc_fe32_parser STATIC
    WHEELTEC_APP/esc_fe32_parser.c
)
target_include_directories(esc_fe32_parser
    PUBLIC
        WHEELTEC_APP/Inc
)
set_target_properties(esc_fe32_parser PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED YES
)
rccar_enable_strict_c_warnings(esc_fe32_parser)

add_library(esc_telemetry STATIC
    WHEELTEC_APP/esc_telemetry.c
)
target_link_libraries(esc_telemetry PUBLIC esc_fe32_parser)
target_include_directories(esc_telemetry
    PUBLIC
        WHEELTEC_APP/Inc
)
set_target_properties(esc_telemetry PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED YES
)
rccar_enable_strict_c_warnings(esc_telemetry)

add_library(esc_soft_uart_rx STATIC
    WHEELTEC_APP/esc_soft_uart_rx.c
)
target_include_directories(esc_soft_uart_rx
    PUBLIC
        WHEELTEC_APP/Inc
)
set_target_properties(esc_soft_uart_rx PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED YES
)
rccar_enable_strict_c_warnings(esc_soft_uart_rx)

add_library(esc_motion_estimator STATIC
    WHEELTEC_APP/esc_motion_estimator.c
)
target_link_libraries(esc_motion_estimator PUBLIC esc_fe32_parser)
target_include_directories(esc_motion_estimator
    PUBLIC
        WHEELTEC_APP/Inc
)
set_target_properties(esc_motion_estimator PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED YES
)
rccar_enable_strict_c_warnings(esc_motion_estimator)

add_library(rc_direction_observer STATIC
    WHEELTEC_APP/rc_direction_observer.c
)
target_include_directories(rc_direction_observer
    PUBLIC
        WHEELTEC_APP/Inc
)
set_target_properties(rc_direction_observer PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED YES
)
rccar_enable_strict_c_warnings(rc_direction_observer)

add_library(mode2_drive_gate STATIC
    WHEELTEC_APP/mode2_drive_gate.c
)
target_link_libraries(mode2_drive_gate PUBLIC esc_motion_estimator)
target_include_directories(mode2_drive_gate
    PUBLIC
        WHEELTEC_APP/Inc
)
set_target_properties(mode2_drive_gate PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED YES
)
rccar_enable_strict_c_warnings(mode2_drive_gate)

add_library(longitudinal_controller STATIC
    WHEELTEC_APP/longitudinal_controller.c
)
target_include_directories(longitudinal_controller
    PUBLIC
        WHEELTEC_APP/Inc
)
set_target_properties(longitudinal_controller PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED YES
)
rccar_enable_strict_c_warnings(longitudinal_controller)

if(RCCAR_BUILD_HOST_TESTS)
    enable_testing()

    add_executable(test_esc_fe32_parser
        tests/host/test_esc_fe32_parser.c
    )
    target_link_libraries(test_esc_fe32_parser PRIVATE esc_fe32_parser)
    set_target_properties(test_esc_fe32_parser PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED YES
    )
    rccar_enable_strict_c_warnings(test_esc_fe32_parser)

    add_test(NAME esc_fe32_parser_unit COMMAND test_esc_fe32_parser)

    add_executable(test_esc_telemetry
        tests/host/test_esc_telemetry.c
    )
    target_link_libraries(test_esc_telemetry PRIVATE esc_telemetry)
    set_target_properties(test_esc_telemetry PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED YES
    )
    rccar_enable_strict_c_warnings(test_esc_telemetry)
    add_test(NAME esc_telemetry_unit COMMAND test_esc_telemetry)

    add_executable(test_esc_soft_uart_rx
        tests/host/test_esc_soft_uart_rx.c
    )
    target_link_libraries(test_esc_soft_uart_rx PRIVATE esc_soft_uart_rx)
    set_target_properties(test_esc_soft_uart_rx PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED YES
    )
    rccar_enable_strict_c_warnings(test_esc_soft_uart_rx)
    add_test(NAME esc_soft_uart_rx_unit COMMAND test_esc_soft_uart_rx)

    add_executable(test_esc_telemetry_stm32
        tests/host/test_esc_telemetry_stm32.c
        WHEELTEC_APP/esc_telemetry_stm32.c
        WHEELTEC_APP/esc_soft_uart_stm32.c
    )
    target_link_libraries(test_esc_telemetry_stm32 PRIVATE
        esc_telemetry
        esc_soft_uart_rx
    )
    target_include_directories(test_esc_telemetry_stm32 PRIVATE tests/host)
    target_compile_definitions(test_esc_telemetry_stm32 PRIVATE
        ESC_TELEMETRY_STM32_HOST_TEST
        ESC_SOFT_UART_STM32_HOST_TEST)
    set_target_properties(test_esc_telemetry_stm32 PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED YES
    )
    rccar_enable_strict_c_warnings(test_esc_telemetry_stm32)
    add_test(NAME esc_telemetry_stm32_unit
             COMMAND test_esc_telemetry_stm32)

    add_executable(test_esc_motion_estimator
        tests/host/test_esc_motion_estimator.c
    )
    target_link_libraries(test_esc_motion_estimator PRIVATE
        esc_motion_estimator
        m
    )
    set_target_properties(test_esc_motion_estimator PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED YES
    )
    rccar_enable_strict_c_warnings(test_esc_motion_estimator)
    add_test(NAME esc_motion_estimator_unit
             COMMAND test_esc_motion_estimator)

    add_executable(test_rc_direction_observer
        tests/host/test_rc_direction_observer.c
    )
    target_link_libraries(test_rc_direction_observer PRIVATE
        rc_direction_observer
    )
    set_target_properties(test_rc_direction_observer PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED YES
    )
    rccar_enable_strict_c_warnings(test_rc_direction_observer)
    add_test(NAME rc_direction_observer_unit
             COMMAND test_rc_direction_observer)

    add_executable(test_mode2_drive_gate
        tests/host/test_mode2_drive_gate.c
    )
    target_link_libraries(test_mode2_drive_gate PRIVATE
        mode2_drive_gate
        m
    )
    set_target_properties(test_mode2_drive_gate PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED YES
    )
    rccar_enable_strict_c_warnings(test_mode2_drive_gate)
    add_test(NAME mode2_drive_gate_unit
             COMMAND test_mode2_drive_gate)

    add_executable(test_longitudinal_controller
        tests/host/test_longitudinal_controller.c
    )
    target_link_libraries(test_longitudinal_controller PRIVATE
        longitudinal_controller
        m
    )
    set_target_properties(test_longitudinal_controller PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED YES
    )
    rccar_enable_strict_c_warnings(test_longitudinal_controller)
    add_test(NAME longitudinal_controller_unit
             COMMAND test_longitudinal_controller)

    add_executable(test_servo_basic_control
        tests/host/test_servo_basic_control.c
        WHEELTEC_APP/servo_basic_control.c
        WHEELTEC_APP/SerialControl_task.c
        WHEELTEC_APP/data_task.c
        WHEELTEC_APP/app_runtime_state.c
    )
    target_include_directories(test_servo_basic_control PRIVATE
        tests/host/stubs
        WHEELTEC_APP/Inc
    )
    target_link_libraries(test_servo_basic_control PRIVATE
        mode2_drive_gate
        longitudinal_controller
        rc_direction_observer
        m
    )
    set_target_properties(test_servo_basic_control PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED YES
    )
    rccar_enable_strict_c_warnings(test_servo_basic_control)
    add_test(NAME servo_basic_control_unit
             COMMAND test_servo_basic_control)

    add_executable(test_soft_uart_irq
        tests/host/test_soft_uart_irq.c
        Core/Src/stm32f4xx_it.c
        WHEELTEC_APP/esc_soft_uart_stm32.c
    )
    target_link_libraries(test_soft_uart_irq PRIVATE esc_soft_uart_rx)
    target_include_directories(test_soft_uart_irq PRIVATE
        tests/host
        tests/host/irq_stubs
        WHEELTEC_APP/Inc
    )
    target_compile_definitions(test_soft_uart_irq PRIVATE
        ESC_SOFT_UART_STM32_HOST_TEST)
    set_target_properties(test_soft_uart_irq PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED YES
    )
    rccar_enable_strict_c_warnings(test_soft_uart_irq)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(test_soft_uart_irq PRIVATE
            -ffunction-sections
            -fdata-sections)
        target_link_options(test_soft_uart_irq PRIVATE
            -Wl,--gc-sections)
    endif()
    add_test(NAME soft_uart_irq_unit COMMAND test_soft_uart_irq)

    if(RCCAR_ENABLE_ESC_FE32_REPLAY)
        message(STATUS "FE32 full replay tests enabled; data root: ${RCCAR_ESC_FE32_REPLAY_ROOT}")

        add_executable(test_esc_fe32_replay
            tests/host/test_esc_fe32_replay.c
        )
        target_link_libraries(test_esc_fe32_replay PRIVATE esc_fe32_parser)
        set_target_properties(test_esc_fe32_replay PROPERTIES
            C_STANDARD 99
            C_STANDARD_REQUIRED YES
        )
        rccar_enable_strict_c_warnings(test_esc_fe32_replay)

        add_executable(test_esc_soft_uart_replay
            tests/host/test_esc_soft_uart_replay.c
        )
        target_link_libraries(test_esc_soft_uart_replay PRIVATE
            esc_fe32_parser
            esc_soft_uart_rx
        )
        set_target_properties(test_esc_soft_uart_replay PROPERTIES
            C_STANDARD 99
            C_STANDARD_REQUIRED YES
        )
        rccar_enable_strict_c_warnings(test_esc_soft_uart_replay)

        function(rccar_read_fe32_fixture_sha macro_name out_var)
            file(STRINGS "${CMAKE_SOURCE_DIR}/tests/fixtures/esc_fe32_samples.h"
                 fixture_sha_lines
                 REGEX "^#define ${macro_name} ")
            if(NOT fixture_sha_lines)
                message(FATAL_ERROR
                    "missing ${macro_name} in tests/fixtures/esc_fe32_samples.h")
            endif()
            list(GET fixture_sha_lines 0 fixture_sha_line)
            string(REGEX REPLACE ".*\"([0-9a-fA-F]+)\".*" "\\1"
                   fixture_sha "${fixture_sha_line}")
            string(LENGTH "${fixture_sha}" fixture_sha_len)
            if(NOT fixture_sha_len EQUAL 64 OR
               NOT fixture_sha MATCHES "^[0-9a-fA-F]+$")
                message(FATAL_ERROR
                    "invalid ${macro_name} in tests/fixtures/esc_fe32_samples.h: ${fixture_sha_line}")
            endif()
            set(${out_var} "${fixture_sha}" PARENT_SCOPE)
        endfunction()

        rccar_read_fe32_fixture_sha(ESC_FE32_FIXTURE_FIRST_SHA256
                                    ESC_FE32_FIXTURE_FIRST_SHA256)
        rccar_read_fe32_fixture_sha(ESC_FE32_FIXTURE_DYNAMIC_01_SHA256
                                    ESC_FE32_FIXTURE_DYNAMIC_01_SHA256)
        rccar_read_fe32_fixture_sha(ESC_FE32_FIXTURE_DYNAMIC_02_SHA256
                                    ESC_FE32_FIXTURE_DYNAMIC_02_SHA256)

        function(rccar_add_fe32_replay_test
                 test_name
                 relative_capture_path
                 expected_sha256
                 expected_bytes
                 expected_frames
                 expected_crc_failures
                 expected_prefix_rejections)
            set(capture_path "${RCCAR_ESC_FE32_REPLAY_ROOT}/${relative_capture_path}")
            add_test(NAME "${test_name}"
                COMMAND "${CMAKE_COMMAND}"
                    "-DREPLAY_EXE=$<TARGET_FILE:test_esc_fe32_replay>"
                    "-DINPUT=${capture_path}"
                    "-DEXPECTED_SHA256=${expected_sha256}"
                    "-DEXPECTED_BYTES=${expected_bytes}"
                    "-DEXPECTED_FRAMES=${expected_frames}"
                    "-DEXPECTED_CRC_FAILURES=${expected_crc_failures}"
                    "-DEXPECTED_PREFIX_REJECTIONS=${expected_prefix_rejections}"
                    -P "${CMAKE_SOURCE_DIR}/cmake/run_esc_fe32_replay.cmake")
        endfunction()

        rccar_add_fe32_replay_test(
            esc_fe32_replay_first
            "2026-09-15_UART_首次上电采集/raw.bin"
            "${ESC_FE32_FIXTURE_FIRST_SHA256}"
            9633 300 0 0)
        rccar_add_fe32_replay_test(
            esc_fe32_replay_dynamic_01
            "2026-09-15_UART_前进刹车后退_01/raw.bin"
            "${ESC_FE32_FIXTURE_DYNAMIC_01_SHA256}"
            109657 3425 0 1)
        rccar_add_fe32_replay_test(
            esc_fe32_replay_dynamic_02
            "2026-09-15_UART_前进刹车后退_02/raw.bin"
            "${ESC_FE32_FIXTURE_DYNAMIC_02_SHA256}"
            108162 3379 0 4)

        function(rccar_add_soft_uart_replay_test
                 test_name
                 relative_capture_path
                 expected_sha256
                 expected_bytes
                 expected_frames
                 expected_crc_failures
                 expected_prefix_rejections)
            set(capture_path "${RCCAR_ESC_FE32_REPLAY_ROOT}/${relative_capture_path}")
            add_test(NAME "${test_name}"
                COMMAND "${CMAKE_COMMAND}"
                    "-DREPLAY_EXE=$<TARGET_FILE:test_esc_soft_uart_replay>"
                    "-DINPUT=${capture_path}"
                    "-DEXPECTED_SHA256=${expected_sha256}"
                    "-DEXPECTED_BYTES=${expected_bytes}"
                    "-DEXPECTED_FRAMES=${expected_frames}"
                    "-DEXPECTED_CRC_FAILURES=${expected_crc_failures}"
                    "-DEXPECTED_PREFIX_REJECTIONS=${expected_prefix_rejections}"
                    -P "${CMAKE_SOURCE_DIR}/cmake/run_esc_fe32_replay.cmake")
        endfunction()

        rccar_add_soft_uart_replay_test(
            esc_soft_uart_replay_first
            "2026-09-15_UART_首次上电采集/raw.bin"
            "${ESC_FE32_FIXTURE_FIRST_SHA256}"
            9633 300 0 0)
        rccar_add_soft_uart_replay_test(
            esc_soft_uart_replay_dynamic_01
            "2026-09-15_UART_前进刹车后退_01/raw.bin"
            "${ESC_FE32_FIXTURE_DYNAMIC_01_SHA256}"
            109657 3425 0 1)
        rccar_add_soft_uart_replay_test(
            esc_soft_uart_replay_dynamic_02
            "2026-09-15_UART_前进刹车后退_02/raw.bin"
            "${ESC_FE32_FIXTURE_DYNAMIC_02_SHA256}"
            108162 3379 0 4)
    else()
        message(STATUS
            "FE32 full replay tests disabled; only host unit tests will run. "
            "Set RCCAR_ENABLE_ESC_FE32_REPLAY=ON and RCCAR_ESC_FE32_REPLAY_ROOT "
            "to enable full real-capture replay.")
    endif()
endif()

if(RCCAR_BUILD_FIRMWARE)
    set(RCCAR_MCU_FLAGS
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
    )

    target_compile_definitions(esc_telemetry PRIVATE USE_HAL_DRIVER STM32F407xx)
    target_include_directories(esc_telemetry PRIVATE
        Core/Inc
        Drivers/STM32F4xx_HAL_Driver/Inc
        Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32F4xx/Include
        Drivers/CMSIS/Include
    )

    set(RCCAR_FIRMWARE_SOURCES
        cmake/startup_stm32f407xx_gcc.s
        Core/Src/main.c
        Core/Src/gpio.c
        Core/Src/freertos.c
        Core/Src/adc.c
        Core/Src/dma.c
        Core/Src/tim.c
        Core/Src/usart.c
        Core/Src/stm32f4xx_it.c
        Core/Src/stm32f4xx_hal_msp.c
        Core/Src/stm32f4xx_hal_timebase_tim.c
        Core/Src/system_stm32f4xx.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
        Middlewares/Third_Party/FreeRTOS/Source/croutine.c
        Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
        Middlewares/Third_Party/FreeRTOS/Source/list.c
        Middlewares/Third_Party/FreeRTOS/Source/queue.c
        Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
        Middlewares/Third_Party/FreeRTOS/Source/tasks.c
        Middlewares/Third_Party/FreeRTOS/Source/timers.c
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
        Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
        Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
        WHEELTEC_BSP/bsp_buzzer.c
        WHEELTEC_BSP/bsp_dwt.c
        WHEELTEC_BSP/bsp_oled.c
        WHEELTEC_BSP/bsp_flash.c
        WHEELTEC_BSP/bsp_adc.c
        WHEELTEC_APP/SerialControl_task.c
        WHEELTEC_APP/show_task.c
        WHEELTEC_APP/data_task.c
        WHEELTEC_APP/app_runtime_state.c
        WHEELTEC_APP/tim_capture_callback.c
        WHEELTEC_APP/uart_callback.c
        WHEELTEC_APP/servo_basic_output.c
        WHEELTEC_APP/servo_basic_control.c
        WHEELTEC_APP/servo_rc_capture.c
        WHEELTEC_APP/hall_speed.c
        WHEELTEC_APP/esc_soft_uart_stm32.c
        WHEELTEC_APP/esc_telemetry_stm32.c
        WHEELTEC_APP/esc_motion_estimator.c
        WHEELTEC_APP/mode2_drive_gate.c
        WHEELTEC_APP/longitudinal_controller.c
    )

    add_executable(WHEELTEC.elf ${RCCAR_FIRMWARE_SOURCES})
    target_link_libraries(WHEELTEC.elf PRIVATE
        esc_telemetry
        esc_soft_uart_rx
        rc_direction_observer
    )
    target_compile_definitions(WHEELTEC.elf PRIVATE USE_HAL_DRIVER STM32F407xx)
    target_include_directories(WHEELTEC.elf PRIVATE
        Core/Inc
        Drivers/STM32F4xx_HAL_Driver/Inc
        Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32F4xx/Include
        Drivers/CMSIS/Include
        Middlewares/Third_Party/FreeRTOS/Source/include
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
        Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
        WHEELTEC_BSP/Inc
        WHEELTEC_APP/Inc
    )
    set_target_properties(WHEELTEC.elf PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED YES
    )
    target_compile_options(WHEELTEC.elf PRIVATE
        ${RCCAR_MCU_FLAGS}
        -ffunction-sections
        -fdata-sections
        -Wall
        -Wextra
    )
    target_compile_options(esc_fe32_parser PRIVATE
        $<$<BOOL:${RCCAR_BUILD_FIRMWARE}>:${RCCAR_MCU_FLAGS}>
    )
    target_compile_options(esc_telemetry PRIVATE
        $<$<BOOL:${RCCAR_BUILD_FIRMWARE}>:${RCCAR_MCU_FLAGS}>
    )
    target_compile_options(esc_soft_uart_rx PRIVATE
        $<$<BOOL:${RCCAR_BUILD_FIRMWARE}>:${RCCAR_MCU_FLAGS}>
    )
    target_compile_options(esc_motion_estimator PRIVATE
        $<$<BOOL:${RCCAR_BUILD_FIRMWARE}>:${RCCAR_MCU_FLAGS}>
    )
    target_compile_options(rc_direction_observer PRIVATE
        $<$<BOOL:${RCCAR_BUILD_FIRMWARE}>:${RCCAR_MCU_FLAGS}>
    )
    target_compile_options(mode2_drive_gate PRIVATE
        $<$<BOOL:${RCCAR_BUILD_FIRMWARE}>:${RCCAR_MCU_FLAGS}>
    )
    target_compile_options(longitudinal_controller PRIVATE
        $<$<BOOL:${RCCAR_BUILD_FIRMWARE}>:${RCCAR_MCU_FLAGS}>
    )
    target_link_options(WHEELTEC.elf PRIVATE
        ${RCCAR_MCU_FLAGS}
        -T${CMAKE_SOURCE_DIR}/cmake/STM32F407VETx_FLASH.ld
        -Wl,-Map=${CMAKE_BINARY_DIR}/WHEELTEC.map
        -Wl,--gc-sections
        --specs=nano.specs
        --specs=nosys.specs
    )

    add_custom_command(TARGET WHEELTEC.elf POST_BUILD
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:WHEELTEC.elf>
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:WHEELTEC.elf> ${CMAKE_BINARY_DIR}/WHEELTEC.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:WHEELTEC.elf> ${CMAKE_BINARY_DIR}/WHEELTEC.bin
    )
endif()
