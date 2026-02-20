#ifndef KSOLVER_H
#define KSOVLER_H

#include "servo2040.h"

#define L1 83.0
#define L2 83.0

void ik_leg(legAngles *bin, double x, double y, double z);

void ik_leg(legAngles *bin, double lx, double ly, double lz, double rx, double ry, double rz);

void ik_polar(legAngles *bin, double r, double x, double th);
void ik_polar(legAngles *bin, double lr, double lx, double lth, double rr, double rx, double rth);

#endif