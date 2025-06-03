// Pimoroni Servo2040 client driver with IMU readback functionality.
// Uses the Adafruit BNO-055 connected to ADC0 (I2C1-SDA) and ADC1 (I2C1-SCL) 

#include "servo2040_client_imu.hpp"
using namespace plasma;
using namespace servo;

ServoCluster servos = ServoCluster(pio0, 0, servo2040::SERVO_1, 18);
WS2812 led_bar(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);

Analog vol_adc = Analog(servo2040::SHARED_ADC, servo2040::VOLTAGE_GAIN);
Analog cur_adc = Analog(servo2040::SHARED_ADC, servo2040::CURRENT_GAIN);
AnalogMux mux = AnalogMux(servo2040::ADC_ADDR_0, servo2040::ADC_ADDR_1, servo2040::ADC_ADDR_2, PIN_UNUSED, servo2040::SHARED_ADC);

BNO055 imu(i2c1, 26, 27);

float servoValues[] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

// Core1 processor running IMU functions and voltage/current readings
void core1_main() {
    bool i = imu.begin();

    if (!i) {
        multicore_fifo_push_blocking(0x0F);
        while(1) sleep_ms(10);
    }

    uint8_t status, self_test, error;
    imu.getSystemStatus(&status, &self_test, &error);

    multicore_fifo_push_blocking(error);

    if (error != 0) {
        while(1) sleep_ms(10);
    }
    
    vector v;
    while(1) {
        
        imu.getVector(VECTOR_EULER, &v);

        uint8_t buffer[3 * sizeof(double)];
        memcpy(&buffer[0], &v.x, sizeof(double));
        memcpy(&buffer[sizeof(double)], &v.y, sizeof(double));
        memcpy(&buffer[2 * sizeof(double)], &v.z, sizeof(double));

        uart_putc(UART_ID, 0b10010000);
        uart_write_blocking(UART_ID, buffer, sizeof(buffer));

        sleep_ms(50); // 20 Hz sensor updates
    }
}

// Core0 processor running UART functions and servo actuation, runs first
int main() {

    stdio_init_all();

    // Initialize LED bar
    led_bar.start();
    led_bar.clear();
    set_led(0b100000, red_t);
    sleep_ms(100);

    // Initialize Servo Cluster
    servos.init();
    set_led(0b110000, red_t);
    sleep_ms(100);

    // Enable UART
    uart_init(UART_ID, 115200);
    gpio_set_function(20, GPIO_FUNC_UART);
    gpio_set_function(21, GPIO_FUNC_UART);
    gpio_pull_up(20);
    gpio_pull_up(21);
    set_led(0b111000, red_t);
    sleep_ms(100);

    // Wait for ESP32 handshake
    while(wait_until_uart() != 0b01101001) {;}
    set_led(0b111100, red_t);
    sleep_ms(100);

    // Enable core1
    multicore_launch_core1(core1_main);
    set_led(0b111110, red_t);
    sleep_ms(100);

    // Check IMU status
    uint32_t success = multicore_fifo_pop_blocking();
    if (success != 0) {
        set_led(0b101010, red_t);
        while(1) sleep_ms(10);
    }
    set_led(0b111111, red_t);
    sleep_ms(500);

    //blink LED bar for visual confirmation
    set_led(0b111111, green_t);
    sleep_ms(500);
    set_led(0x00, green_t);
    sleep_ms(500);
    set_led(0b111111, green_t);
    sleep_ms(500);
    set_led(0x00, green_t);

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
            // send back voltage and current (0b1111)
            //return_sensor();
        } else if (returnType == 0b1001) {
            // send back imu data
            //return_imu();
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
    set_led(dataPacket, red_t);

}

void cmd_enable() {

    set_servos();
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

void set_led(int pinMask, color_t color) {

    for (int i = 0; i < 6; i++) {

        uint8_t mask = 1 << i;
        if (pinMask & mask) {
            led_bar.set_rgb(i,
                (color == red_t) ? 64 : 0,
                (color == green_t) ? 64 : 0,
                (color == blue_t) ? 64 : 0
            );
        } else {
            led_bar.set_rgb(i, 0, 0, 0);
        }
        // led_bar.set_hsv(i, 171.0f, 1.0f, (pinMask & mask) ? 0.1f: 0.0f);

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
        // sleep_ms(delay / 100);
        sleep_us(delay * 1000 / 100);
    }
}