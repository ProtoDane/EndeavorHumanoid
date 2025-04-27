#Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Include build functions Pico / Pimoroni SDK
include($ENV{PIMORONI_PICO_PATH}/pico_sdk_import.cmake)
include($ENV{PIMORONI_PICO_PATH}/pimoroni_pico_import.cmake)

# Configure Project
project(servo2040_client_imu C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(${PROJECT_NAME} servo2040_client_imu.cpp)

include($ENV{PIMORONI_PICO_PATH}/libraries/servo2040/servo2040.cmake)
include($ENV{PIMORONI_PICO_PATH}/drivers/button/button.cmake)
include($ENV{PIMORONI_PICO_PATH}/drivers/analog/analog.cmake)
include($ENV{PIMORONI_PICO_PATH}/drivers/analogmux/analogmux.cmake)
include(C:/VSARM/lib/rp2040_bno055/rp2040_bno055.cmake)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    servo2040
    button
    analog
    analogmux
    pico_multicore
    rp2040_bno055
)

target_compile_definitions(${PROJECT_NAME} PRIVATE 
    PICO_DEFAULT_UART=1
    PICO_DEFAULT_UART_TX_PIN=20
    PICO_DEFAULT_UART_RX_PIN=21
)

pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

pico_add_extra_outputs(${PROJECT_NAME})