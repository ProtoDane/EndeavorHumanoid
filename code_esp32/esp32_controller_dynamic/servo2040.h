#ifndef SERVO2040_H
#define SERVO2040_H

#include <HardwareSerial.h>

#include "config.h"
// #include "movesets.h"
#include "kinematics.h"

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

float idleAngles[] = {
  0.0, 
  -90.0, -60.0, 0.0, -20.0, 
  -4.0, -49.3, 31.2, -4.0, 
  4.0, 49.3, -31.2, 4.0, 
  90.0, 60.0, 0.0, 20.0
};

// UART Instance
HardwareSerial servoSerial(2);

// Data structure to hold info for a single servo
struct servoStruct {
  int min;
  int mid;
  int max;
  bool angle180; // if false, will map to 270 angle range
};

// Create servoStructs for each calibrated servo.  If working with multiple units, apply the config based on DEVICE_SELECT
#if defined(ENDEAVOR)
  servoStruct s_torso = {515, 1458, 2397, false};   // TORSO
  servoStruct s_ra1 =   {510, 1495, 2397, false};   // RIGHT ARM SHOULDER BASE
  servoStruct s_ra2 =   {543, 1505, 2413, false};   // RIGHT ARM SHOULDER SWING
  servoStruct s_ra3 =   {592, 1495, 2404, true};    // RIGHT ARM BICEP SWIVEL
  servoStruct s_ra4 =   {593, 1493, 2385, true};    // RIGHT ARM ELBOW
  servoStruct s_rl1 =   {592, 1512, 2405, true};    // RIGHT LEG THIGH
  servoStruct s_rl2 =   {588, 1490, 2372, true};    // RIGHT LEG FEMUR
  servoStruct s_rl3 =   {643, 1550, 2432, true};    // RIGHT LEG TIBIA
  servoStruct s_rl4 =   {587, 1493, 2405, true};    // RIGHT LEG ANKLE
  servoStruct s_la1 =   {505, 1457, 2409, false};   // LEFT ARM SHOULDER BASE
  servoStruct s_la2 =   {550, 1509, 2453, false};   // LEFT ARM SHOULDER SWING
  servoStruct s_la3 =   {622, 1530, 2480, true};    // LEFT ARM BICEP SWIVEL
  servoStruct s_la4 =   {622, 1533, 2440, true};    // LEFT ARM ELBOW
  servoStruct s_ll1 =   {597, 1500, 2410, true};    // LEFT LEG THIGH
  servoStruct s_ll2 =   {685, 1580, 2470, true};    // LEFT LEG FEMUR
  servoStruct s_ll3 =   {567, 1522, 2437, true};    // LEFT LEG TIBIA
  servoStruct s_ll4 =   {591, 1520, 2428, true};    // LEFT LEG ANKLE

#elif defined(CONTENDER)
  servoStruct s_torso = {524, 1470, 2405, false};   // TORSO
  servoStruct s_ra1 =   {575, 1528, 2456, false};   // RIGHT ARM SHOULDER BASE
  servoStruct s_ra2 =   {542, 1522, 2420, false};   // RIGHT ARM SHOULDER SWING
  servoStruct s_ra3 =   {601, 1513, 2428, true};    // RIGHT ARM BICEP SWIVEL
  servoStruct s_ra4 =   {607, 1520, 2430, true};    // RIGHT ARM ELBOW
  servoStruct s_rl1 =   {653, 1558, 2451, true};    // RIGHT LEG THIGH
  servoStruct s_rl2 =   {597, 1530, 2440, true};    // RIGHT LEG FEMUR
  servoStruct s_rl3 =   {584, 1500, 2404, true};    // RIGHT LEG TIBIA
  servoStruct s_rl4 =   {560, 1486, 2389, true};    // RIGHT LEG ANKLE
  servoStruct s_la1 =   {561, 1520, 2440, false};   // LEFT ARM SHOULDER BASE
  servoStruct s_la2 =   {557, 1511, 2447, false};   // LEFT ARM SHOULDER SWING
  servoStruct s_la3 =   {602, 1514, 2413, true};    // LEFT ARM BICEP SWIVEL
  servoStruct s_la4 =   {615, 1537, 2456, true};    // LEFT ARM ELBOW
  servoStruct s_ll1 =   {555, 1488, 2398, true};    // LEFT LEG THIGH
  servoStruct s_ll2 =   {587, 1512, 2415, true};    // LEFT LEG FEMUR
  servoStruct s_ll3 =   {587, 1508, 2400, true};    // LEFT LEG TIBIA
  servoStruct s_ll4 =   {605, 1509, 2416, true};    // LEFT LEG ANKLE
#endif

// Pin mapping array for servos: 0th index = SERVO1, 17th index = SERVO18
// IT IS NOT RECOMMENDED THAT YOU MODIFY THIS ARRAY
servoStruct servoCluster[] = {
  s_torso, s_ra1, s_ra2, s_ra3, s_ra4,
  s_rl1, s_rl2, s_rl3, s_rl4,
  s_ll1, s_ll2, s_ll3, s_ll4,
  s_la1, s_la2, s_la3, s_la4,
};

// Send first byte packet of a message to the Servo2040.  First 4 bits are a preamble, last 4 bits are a command
void sendCommand(int preamble, int cmd) {
  servoSerial.write(preamble << 4 | cmd);
}

// Send pulse in microseconds as two bytes for a single servo channel
// FOR INTERNAL USE, DO NOT USE THIS FUNCTION FOR YOUR OWN CODE
void setServo(int pulse) {
  int first = pulse / 100;
  int second = pulse % 100;
  servoSerial.write(first);
  servoSerial.write(second);
}

// Maps the desired angle to the calibrated pulse.  angle180 used to differentiate between 180 and 270 deg servos
int mapPulse(float angle, float min, float mid, float max, bool angle180) {

  float maxAngle = angle180 ? 90.0 : 135.0;

  if (angle > 0.0) {

    return (int) (angle * (max - mid) / maxAngle + mid);

  } else if (angle < 0.0) {

    return (int) ((angle + maxAngle) * (mid - min) / maxAngle + min);

  } else {

    return mid;

  }
}

// Sets the servo cluster to the desired list of angles.  pinMask used to identify which servos are to be actuated
// NOTE1: LENGTH OF angles[] AND NUMBER OF BINARY 1'S IN pinMask ARE ASSUMED TO BE EQUAL
// NOTE2: THE BINARY ORDER OF pinMask FROM RIGHT TO LEFT CORRESPONDS TO THE SERVO CHANNELS ON THE SERVO2040 BOARD FROM SERVO1 TO SERVO18
void setServoCluster(float angles[], int pinMask) {
  
  int pIndex = 0; // angles array index (assumes array length and number of 1's in pinMask are equal)
  
  for (int i = 0; i < 18; i++) {

    uint32_t mask = 1 << i; // step forward mask by 1 bit

    if (pinMask & mask) { // true if i-th pinMask bit is equal to 1

      servoStruct servo = servoCluster[i];
      int calibratedPulse = mapPulse(angles[pIndex], servo.min, servo.mid, servo.max, servo.angle180);
      setServo(calibratedPulse);
      pIndex++;

    } else {

      // Set servo to an out of range pulse; will be ignored by Servo2040
      setServo(1);
    
    }
  }
}

// Implmentation of setServoCluster() that takes in a legAngles and armAngles structs instead of an angle array.
// Intended for use with IK functions, requires all servos to have a defined joint angle (pinMask = ALL_SERVOS)
void setServoCluster2(legAngles *l, armAngles *a, float torsoAngle) {
  float angles[17];
  angles[0] = torsoAngle;
  angles[1] = a->ra1;
  angles[2] = a->ra2;
  angles[3] = a->ra3;
  angles[4] = a->ra4;
  angles[5] = l->rth1;
  angles[6] = l->rth2;
  angles[7] = l->rth3;
  angles[8] = l->rth1;
  angles[9] = l->lth1;
  angles[10] = l->lth2;
  angles[11] = l->lth3;
  angles[12] = l->lth1;
  angles[13] = a->la1;
  angles[14] = a->la2;
  angles[15] = a->la3;
  angles[16] = a->la4;

  setServoCluster(angles, ALL_SERVOS);
}

void setServoSequence(int n, float sequence[], int pinMask, int sequenceLength) {

  // Count the servos being actuated in pinMask
  int servoCount = 0;
  for (int i = 0; i < 18; i++) {

    uint32_t mask = 1 << i;
    if (pinMask & mask) {servoCount++;}

  }

  // Check servos once again and set their pulse to its respective servo in the n-th keyframe
  int p = 0;
  for (int i = 0; i < 18; i++) {

    uint32_t mask = 1 << i;
    if (pinMask & mask) {

      servoStruct servo = servoCluster[i];
      int calibratedPulse = mapPulse(sequence[(n * servoCount + p) % sequenceLength], 
        servo.min, servo.mid, servo.max, servo.angle180);

      setServo(calibratedPulse);
      p++;

    } else {
      setServo(1);
    }
  }
}

void setServoDelay(float angles[], int pinMask, int delayMs) {
  int first = delayMs / 100;
  int second = delayMs % 100;
  servoSerial.write(first);
  servoSerial.write(second);

  setServoCluster(angles, pinMask);
  delay(delayMs);
}


void cmdEnable() {
  Serial.println("Enabling Servo2040...");
  sendCommand(RETURN_NONE, CMD_ENABLE);
  setServoCluster(idleAngles, ALL_SERVOS); //0b11110000000011111

  delay(500);
}

void cmdDisable() {
  Serial.println("Disabling Servo2040...");
  sendCommand(RETURN_NONE, CMD_DISABLE);
}

#endif