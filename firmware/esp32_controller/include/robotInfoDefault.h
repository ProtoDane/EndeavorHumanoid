#ifndef RINFO_DEFAULT
#define RINFO_DEFAULT

// Data structure to hold info for a single servo
struct servoStruct {
    int min;
    int mid;
    int max;
    bool angle180; // if false, will map to 270 angle range
};

const servoStruct s_torso = {515, 1458, 2397, false};   // TORSO
const servoStruct s_ra1 =   {510, 1495, 2397, false};   // RIGHT ARM SHOULDER BASE
const servoStruct s_ra2 =   {543, 1505, 2413, false};   // RIGHT ARM SHOULDER SWING
const servoStruct s_ra3 =   {592, 1495, 2404, true};    // RIGHT ARM BICEP SWIVEL
const servoStruct s_ra4 =   {593, 1493, 2385, true};    // RIGHT ARM ELBOW
const servoStruct s_rl1 =   {592, 1512, 2405, true};    // RIGHT LEG THIGH
const servoStruct s_rl2 =   {588, 1490, 2372, true};    // RIGHT UPPER LEG
const servoStruct s_rl3 =   {643, 1550, 2432, true};    // RIGHT LOWER LEG
const servoStruct s_rl4 =   {587, 1493, 2405, true};    // RIGHT LEG ANKLE
const servoStruct s_la1 =   {505, 1457, 2409, false};   // LEFT ARM SHOULDER BASE
const servoStruct s_la2 =   {550, 1509, 2453, false};   // LEFT ARM SHOULDER SWING
const servoStruct s_la3 =   {622, 1530, 2480, true};    // LEFT ARM BICEP SWIVEL
const servoStruct s_la4 =   {622, 1533, 2440, true};    // LEFT ARM ELBOW
const servoStruct s_ll1 =   {597, 1500, 2410, true};    // LEFT LEG THIGH
const servoStruct s_ll2 =   {685, 1580, 2470, true};    // LEFT UPPER LEG
const servoStruct s_ll3 =   {567, 1522, 2437, true};    // LEFT LOWER LEG
const servoStruct s_ll4 =   {591, 1520, 2428, true};    // LEFT LEG ANKLE

const servoStruct servoCluster[] = {
  s_torso, s_ra1, s_ra2, s_ra3, s_ra4,
  s_rl1, s_rl2, s_rl3, s_rl4,
  s_ll1, s_ll2, s_ll3, s_ll4,
  s_la1, s_la2, s_la3, s_la4,
};

#endif