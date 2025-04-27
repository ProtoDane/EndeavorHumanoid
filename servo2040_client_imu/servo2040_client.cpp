#include "servo2040_client.hpp"
using namespace plasma;
using namespace servo;

ServoCluster servos = ServoCluster(pio0, 0, servo2040::SERVO_1, 18);
WS2812 led_bar(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);

Analog vol_adc = Analog(servo2040::SHARED_ADC, servo2040::VOLTAGE_GAIN);
Analog cur_adc = Analog(servo2040::SHARED_ADC, servo2040::CURRENT_GAIN);
AnalogMux mux = AnalogMux(servo2040::ADC_ADDR_0, servo2040::ADC_ADDR_1, servo2040::ADC_ADDR_2, PIN_UNUSED, servo2040::SHARED_ADC);

float servoValues[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

int main() {
    
    sleep_ms(100);

    stdio_init_all();

    // Initialize Servo Cluster
    servos.init();

    // Initialize LED bar
    led_bar.start();
    led_bar.clear();

    // Enable UART
    uart_init(UART_ID, 115200);
    gpio_set_function(20, GPIO_FUNC_UART);
    gpio_set_function(21, GPIO_FUNC_UART);
    gpio_pull_up(20);
    gpio_pull_up(21);

    // Configure relay pin

    //blink LED bar
    set_led(0b111111);
    sleep_ms(500);
    set_led(0b1000000);
    sleep_ms(500);
    set_led(0b111111);
    sleep_ms(500);
    set_led(0b1000000);

    cmd_disable();

    // Main Loop
    while(1) {
        
        uint8_t uartPacket = wait_until_uart();
        
        uint8_t returnType = (uartPacket & 0xF0) >> 4;
        uint8_t commandSelect = uartPacket & 0x0F;

        bool cmdSuccess = true;
        if (returnType == 0x08) {
            // no return message sent (0b1000)

        } else if (returnType == 0x0F) {
            // send back voltage and current ()
            return_sensor();

        } else {
            // packet is invalid
            cmdSuccess = false;
        }

        if (cmdSuccess) {
            switch(commandSelect) {

                case CMD_ENABLE:
                    cmd_enable();
                    break;
                
                case CMD_DISABLE:
                    cmd_disable();
                    break;
    
                case CMD_PULSE:

                    set_servos();
                    break;
                case CMD_PULSE_DELAY:

                    set_servos_delay();
                    break;

                case CMD_LED:
                    //Set the LED state
                    cmd_set_led();
                    break;
    
                default:
                    //No command detected
                    break;
            }
        }
    }
}

uint8_t wait_until_uart() {

    while(!uart_is_readable(UART_ID)) {;}

    return uart_getc(UART_ID);
}

void cmd_set_led() {

    uint8_t dataPacket = wait_until_uart();
    set_led(dataPacket);

}

void cmd_enable() {

    // servos.enable_all();
    set_servos(); //
}

void cmd_disable() {

    servos.disable_all();
}

void return_sensor() {
    
    mux.select(servo2040::VOLTAGE_SENSE_ADDR);
    float v = vol_adc.read_voltage();
    
    mux.select(servo2040::CURRENT_SENSE_ADDR);
    float i = cur_adc.read_current();

    int v_int = (int)(v * 100.0);
    int i_int = (int)(i * 100.0);

    uart_putc(UART_ID, 0b11110000);
    uart_putc(UART_ID, (uint8_t)(v_int / 100));
    uart_putc(UART_ID, (uint8_t)(v_int % 100));
    uart_putc(UART_ID, (uint8_t)(i_int / 100));
    uart_putc(UART_ID, (uint8_t)(i_int % 100));
}

void set_led(int values) {

    for (int i = 0; i < 6; i++) {

        uint8_t mask = 1 << i;
        led_bar.set_hsv(i, 171.0f, 1.0f, (values & mask) ? 0.1f: 0.0f);
    }
}

void set_servos() {

    for (int i = 0; i < servo2040::NUM_SERVOS; i++) {
        
        uint8_t firstVal = wait_until_uart();
        uint8_t secondVal = wait_until_uart();
        float pulse = (float) firstVal * 100.0f + (float) secondVal;
        
        if (pulse <= SERVO_MAX && pulse >= SERVO_MIN) {
            servos.pulse(i, pulse, true);
            servoValues[i] = pulse;
        }
    }
}

void set_servos_delay() {

    uint32_t servoMask = 0;
    float pulseValues[18];

    uint8_t firstVal = wait_until_uart();
    uint8_t secondVal = wait_until_uart();
    int delay = (int) firstVal * 100 + (int) secondVal; //ms

    for (int i = 0; i < servo2040::NUM_SERVOS; i++) {

        uint8_t firstVal = wait_until_uart();
        uint8_t secondVal = wait_until_uart();
        float pulse = (float) firstVal * 100.0f + (float) secondVal;
        
        if (pulse <= SERVO_MAX && pulse >= SERVO_MIN) {
            servoMask = servoMask | (1 << i);
            pulseValues[i] = pulse;
        }
    }

    for (int t = 0; t < 100; t++) {

        for (int i = 0; i < 18; i++) {

            if (servoMask & (1 << i)) {

                float newPulse = servoValues[i] + (pulseValues[i] - servoValues[i]) * (t+1) / 100;
                servos.pulse(i, newPulse, true);

                if (t == 99) {servoValues[i] = newPulse;}

            }
        }
        sleep_ms(delay / 100);
    }
}