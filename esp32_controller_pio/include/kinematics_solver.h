#ifndef KSOLVER_H
#define KSOVLER_H

#include "servo2040.h"

#define L1 83.0
#define L2 83.0

void ik_leg(legAngles *bin, double x, double y, double z);

void ik_legs(legAngles *bin, double lx, double ly, double lz, double rx, double ry, double rz);

#endif