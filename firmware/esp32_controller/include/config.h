#ifndef MAIN_H
#define MAIN_H

// Voltage cutoff threshold to disable control when battery is low.
// UNUSED DUE TO SERVO2040 VOLTAGE SENSOR BEING TOO NOISY
#define VOLTAGE_CUTOFF  7.20

// INT VALUE TO COMPARE AGAINST TO PERFORM MOVEMENT ACTIONS USING
// LEFT AND RIGHT JOYSTICKS
#define AXIS_THRESHOLD 400

// Enum to determine which IMU sensor is selected (DISABLED = no IMU, 055 = BNO055, 08X = BNO080/085 on ESP32 UART)
enum IMU_SELECT {IMU_DISABLED, IMU_055, IMU_08X};

#ifndef IMU_CONFIG
    #define IMU_CONFIG IMU_DISABLED
#endif

// Fall protection settings.  Tip threshold determines a what pitch angle to trigger fall protection.
#define IMU_TIP_THRESHOLD 30.0

#endif