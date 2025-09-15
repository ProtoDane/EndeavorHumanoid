#ifndef KINEMATICS_H
#define KINEMATICS_H

// Leg linkage lengths.  Do not change unless using your own design
#define L1 83.0
#define L2 83.0

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

// Compute the inverse kinematics for a single leg
void ik_leg(legAngles *bin, double x, double y, double z) {
  
  double th1 = atan(y / z);
  double r3 = sqrt((x*x) + (y*y) + (z*z));

  // Check if parameters violate the Triangle Inequality Theorem: |L1 - L2| < r3 < L1 + L2
  if (r3 > L1 + L2 || r3 < fabs(L1 - L2)) {
    bin->success = false;
    return;
  }

  double ph3 = acos((L1*L1 + L2*L2 - r3*r3) / (2*L1*L2));
  double ph1 = atan(x / sqrt(z*z + y*y));
  double ph2 = acos((L2*L2 + r3*r3 - L1*L1) / (2*L2*r3));

  double th2 = ph3 + ph2 - ph1 - 0.5*PI;
  double th3 = ph3 - th2;

  // Convert to radians and transform to servo's reference
  bin->lth1 = th1 * 180.0 / PI;
  bin->lth2 = 90.0 - th2 * 180.0 / PI;
  bin->lth3 = th3 * 180.0 / PI - 90.0;
  bin->success = true;
}

// Compute the inverse kinematics for left and right legs
void ik_legs(legAngles *bin, double lx, double ly, double lz, double rx, double ry, double rz) {
  legAngles tmp;

  // Left leg IK
  ik_leg(&tmp, lx, ly, lz);
  if (tmp.success) {
    bin->lth1 = tmp.lth1;
    bin->lth2 = tmp.lth2;
    bin->lth3 = tmp.lth3;
  } else {
    bin->success = false;
    return;
  }

  // Right leg IK (mirrored from left leg, so multiply by -1)
  ik_leg(&tmp, rx, ry, rz);
  if (tmp.success) {
    bin->rth1 = -1.0 * tmp.lth1;
    bin->rth2 = -1.0 * tmp.lth2;
    bin->rth3 = -1.0 * tmp.lth3;
  } else {
    bin->success = false;
    return;
  }

  bin->success = true;
}

#endif