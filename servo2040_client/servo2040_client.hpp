#pragma once

// Standard C Libraries
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

// Pico SDK Libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/uart.h"

// Pimoroni Libraries
#include "servo2040.hpp"
#include "button.hpp"
#include "analogmux.hpp"
#include "analog.hpp"

// User Libraries
#include "rp2040_bno055.hpp"

// BNO-055 IMU is connected to 2040 via I2C?  Set to true if yes.
#define IMU_ENABLED true

// UART port for ESP32 communication
#define UART_ID uart1

// Command code macros
#define CMD_PULSE           0x07
#define CMD_PULSE_DELAY     0x06
#define CMD_LED             0x02
#define CMD_ENABLE          0x0F
#define CMD_DISABLE         0x01 

// Servo pulse range
#define SERVO_MAX   2500.0f
#define SERVO_MIN   500.0f

// RGB LED color
typedef enum{
    red_t, blue_t, green_t
} color_t;

// Function prototypes
uint8_t wait_until_uart(void);

void return_sensor(void);
void return_imu(void);

void cmd_set_led(void);
void cmd_enable(void);
void cmd_disable(void);

void set_servos(void);
void set_servos_delay(void);
void set_led(int mask, color_t color);