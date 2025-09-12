#ifndef MAIN_H
#define MAIN_H

// Voltage cutoff threshold to disable control when battery is low.
// UNUSED DUE TO SERVO2040 VOLTAGE SENSOR BEING TOO NOISY
#define VOLTAGE_CUTOFF  7.20

// SAFETY LOCK FOR FORWARD ROLL (X-BUTTON)
// SET TO TRUE TO PREVENT MISFIRES AND ACCIDENTS
#define ULT_LOCK true

// INT VALUE TO COMPARE AGAINST TO PERFORM MOVEMENT ACTIONS USING
// LEFT AND RIGHT JOYSTICKS
#define AXIS_THRESHOLD 400

// DISABLES RELAY ACTIVATION TO PREVENT SERVO ACTUATION.  INTENDED
// FOR USAGE WITH A SERIAL MONITOR FOR DEBUGGING PURPOSES
#define SERIAL_DEBUG_MODE false

// Fall protection settings.  Tip threshold determines a what pitch angle to trigger fall protection.
enum IMU_SELECT {IMU_DISABLED, IMU_055, IMU_08X};
#define IMU_TIP_THRESHOLD 30.0
#define FALL_PROTECTION_ENABLED true

// If you're working with multiple robots, you can use this to select between
// devices and incorporate their respective calibration and controller mac
// addresses
#define ENDEAVOR

// Use DEVICE_SELECT to bind each controller's MAC address to their respective unit
#if defined(ENDEAVOR)
  #define CTRL_MAC "98:b6:7f:0a:fa:59"
  #define IMU_CONFIG IMU_055
#elif defined(CONTENDER)
  #define CTRL_MAC "98:b6:d5:20:65:22"
  #define IMU_CONFIG  IMU_08X
#endif

#endif