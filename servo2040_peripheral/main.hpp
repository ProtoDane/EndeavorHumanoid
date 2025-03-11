#pragma once

#include <stdio.h>
#include "pico/stdlib.h"
#include "servo2040.hpp"
#include "button.hpp"
#include "hardware/uart.h"
#include "analogmux.hpp"
#include "analog.hpp"

#define UART_ID uart1

#define CMD_PULSE           0x07
#define CMD_LED             0x02
#define CMD_ENABLE          0x0F
#define CMD_DISABLE         0x01 

#define SERVO_MAX   2500.0f
#define SERVO_MIN   500.0f

typedef enum {
    NONE, PULSE, ENABLE, DISABLE, LED
} cmdSelect;

uint8_t wait_until_uart(void);

void cmd_set_led(void);
void cmd_enable(void);
void cmd_disable(void);

void set_servos(void);