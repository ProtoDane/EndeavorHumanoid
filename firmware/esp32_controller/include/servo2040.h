#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "config.h"
#include "presets.h"

#ifdef CONFIG_FILE
    #define STRINGIFY(x) #x
    #define TOSTRING(x) STRINGIFY(x)
    #pragma message "FUCK MY CHUNGUS LIFE"
    #include TOSTRING(CONFIG_FILE)
#else
    #pragma message "Using robotInfoDefault.h"
    #include "robotInfoDefault.h"
    #pragma message "Using robotInfoDefault.h"
#endif

// PREAMBLES
#define RETURN_NONE 0x08
#define RETURN_VA   0x0F
#define RETURN_IMU  0b1001

// COMMAND CODES
#define CMD_PULSE       0x07
#define CMD_PULSE_DELAY 0x06
#define CMD_LED         0x02
#define CMD_ENABLE      0x0F
#define CMD_DISABLE     0x01
#define CMD_NONE        0x00

// COMMONLY USED PIN MASKS
#define ALL_SERVOS  0x1FFFF
#define LEG_SERVOS  0b1111111100000
#define RIGHT_ARM   0b11110
#define LEFT_ARM    0b11110000000011110

struct legAngles {
    double lth1;
    double lth2;
    double lth3;
    double rth1;
    double rth2;
    double rth3;
    bool success;
};

struct armAngles {
    double la1;
    double la2;
    double la3;
    double la4;
    double ra1;
    double ra2;
    double ra3;
    double ra4;
    bool success;
};

class servoHandler {
    public:


        bool begin(QueueHandle_t serialQueue);
        void actuate(double pidOutput);

        void sendCommand(int preamble, int cmd);
        void setServo(int pulse);

        void setServoCluster(const float* angles, int pinMask);
        void setServoCluster(legAngles *l, armAngles *a, float torsoAngle);
        void setServoCluster(legAngles *l, armAngles *a, float lth4, float rth4, float torsoAngle);

        void setServoSequence(int n, const float *sequence, int pinMask, int sequenceLength);

        void setServoDelay(const float *angles , int pinMask, int delayMs);

        void cmdEnable();
        void cmdDisable();

    private:
        QueueHandle_t _serialQueue;

        int mapPulse(float angle, float min, float mid, float max, bool angle180);
        void queueWrite(int msg);
};

#endif