#ifndef MAIN_H
#define MAIN_H

#define CMD_PULSE   0x07
#define CMD_LED     0x02
#define CMD_ENABLE  0x0F
#define CMD_DISABLE 0x01

#define RETURN_NONE 0x08
#define RETURN_VA   0x0F

struct servoStruct {
  int min;
  int mid;
  int max;
  bool angle180; // if false, will assume 270 angle range
};

// define servo's calibration parameters
servoStruct s_torso = {515, 1458, 2397, false};
servoStruct s_ra1 =   {510, 1495, 2397, false};
servoStruct s_ra2 =   {543, 1505, 2413, false};
servoStruct s_ra3 =   {592, 1495, 2404, true};
servoStruct s_ra4 =   {593, 1493, 2385, true};
servoStruct s_rl1 =   {592, 1512, 2405, true};
servoStruct s_rl2 =   {588, 1490, 2372, true};
servoStruct s_rl3 =   {643, 1550, 2432, true};
servoStruct s_rl4 =   {587, 1493, 2405, true};
servoStruct s_la1 =   {505, 1457, 2409, false};
servoStruct s_la2 =   {550, 1509, 2453, false};
servoStruct s_la3 =   {622, 1530, 2480, true};
servoStruct s_la4 =   {622, 1533, 2440, true};
servoStruct s_ll1 =   {633, 1520, 2440, true};
servoStruct s_ll2 =   {685, 1580, 2470, true};
servoStruct s_ll3 =   {567, 1522, 2437, true};
servoStruct s_ll4 =   {591, 1520, 2428, true};

// define pin mapping array for servos: 0th index = SERVO1, 17th index = SERVO18
servoStruct servoCluster[] = {
  s_torso, s_ra1, s_ra2, s_ra3, s_ra4,
  s_rl1, s_rl2, s_rl3, s_rl4,
  s_la1, s_la2, s_la3, s_la4,
  s_ll1, s_ll2, s_ll3, s_ll4,
  };

float idleAngles[] = {0.0, 90.0, 60.0, 0.0, 20.0, -90.0, -60.0, 0.0, -20.0};
const int idleMask = 0b11110000000011111;

float crouchAngles[] = {0.0, 45.0, 30.0, 0.0, 20.0};
const int crouchMask = 0b11111;

#endif