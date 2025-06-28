#ifndef MAIN_H
#define MAIN_H

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

#define IMU_TIP_THRESHOLD 30.0
#define FALL_PROTECTION_ENABLED true

// If you're working with multiple robots, you can use this to select between
// devices and incorporate their respective calibration and controller mac
// addresses
enum DeviceID {E20A, C25A};
#define DEVICE_SELECT E20A

// Use DEVICE_SELECT to bind each controller's MAC address to their respective unit
#if DEVICE_SELECT == E20A
  #define CTRL_MAC "98:b6:7f:0a:fa:59"

#elif DEVICE_SELECT == C25A
  #define CTRL_MAC "98:b6:d5:20:65:22"
#endif

#endif