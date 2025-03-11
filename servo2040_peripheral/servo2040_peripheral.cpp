#include "main.hpp"
using namespace plasma;
using namespace servo;

ServoCluster servos = ServoCluster(pio0, 0, servo2040::SERVO_1, 18);
WS2812 led_bar(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);

Analog vol_adc = Analog(servo2040::SHARED_ADC, servo2040::VOLTAGE_GAIN);
Analog cur_adc = Analog(servo2040::SHARED_ADC, servo2040::CURRENT_GAIN);
AnalogMux mux = AnalogMux(servo2040::ADC_ADDR_0, servo2040::ADC_ADDR_1, servo2040::ADC_ADDR_2, PIN_UNUSED, servo2040::SHARED_ADC);

int main() {
    
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
    gpio_init(servo2040::ADC0);
    gpio_set_dir(servo2040::ADC0, GPIO_OUT);
    gpio_put(servo2040::ADC0, 0);

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
                    //
                    set_servos();
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

    for (int i = 0; i < 6; i++) {

        uint8_t mask = 1 << i;
        led_bar.set_hsv(i, 171.0f, 1.0f, (dataPacket & mask) ? 0.1f: 0.0f);
    }
}

void cmd_enable() {

    gpio_put(servo2040::ADC0, 1);
    set_servos();
}

void cmd_disable() {

    gpio_put(servo2040::ADC0, 0);
    servos.disable_all();
}

void set_servos() {

    for (int i = 0; i < servo2040::NUM_SERVOS; i++) {
        
        uint8_t firstVal = wait_until_uart();
        uint8_t secondVal = wait_until_uart();
        float pulse = (float) firstVal * 100.0f + (float) secondVal;
        
        if (pulse <= SERVO_MAX && pulse >= SERVO_MIN) {
            servos.pulse(i, pulse, true);
        }
    }
}