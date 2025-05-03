#pragma once

// Standard C Libraries
#include <stdio.h>
#include <string.h>

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
#include "RP2040Atomic.hpp"

#define UART_ID uart1

#define CMD_PULSE           0x07
#define CMD_PULSE_DELAY     0x06
#define CMD_LED             0x02
#define CMD_ENABLE          0x0F
#define CMD_DISABLE         0x01 


#define SERVO_MAX   2500.0f
#define SERVO_MIN   500.0f

typedef enum{
    red_t, blue_t, green_t
} color_t;

uint8_t wait_until_uart(void);

void return_sensor(void);
void return_imu(void);

void cmd_set_led(void);
void cmd_enable(void);
void cmd_disable(void);

void set_servos(void);
void set_servos_delay(void);
void set_led(int mask, color_t color);