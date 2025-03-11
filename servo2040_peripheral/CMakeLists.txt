#Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Include build functions Pico / Pimoroni SDK
include($ENV{PIMORONI_PICO_PATH}/pico_sdk_import.cmake)
include($ENV{PIMORONI_PICO_PATH}/pimoroni_pico_import.cmake)

# Configure Project
project(servo2040_peripheral C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(${PROJECT_NAME} servo2040_peripheral.cpp)

include($ENV{PIMORONI_PICO_PATH}/libraries/servo2040/servo2040.cmake)
include($ENV{PIMORONI_PICO_PATH}/drivers/button/button.cmake)
include($ENV{PIMORONI_PICO_PATH}/drivers/analog/analog.cmake)
include($ENV{PIMORONI_PICO_PATH}/drivers/analogmux/analogmux.cmake)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    servo2040
    button
    analog
    analogmux
)

target_compile_definitions(${PROJECT_NAME} PRIVATE 
    PICO_DEFAULT_UART=1
    PICO_DEFAULT_UART_TX_PIN=20
    PICO_DEFAULT_UART_RX_PIN=21
)

# enable usb output, disable uart output (so it doesn't confuse any connected servos)
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

pico_add_extra_outputs(${PROJECT_NAME})