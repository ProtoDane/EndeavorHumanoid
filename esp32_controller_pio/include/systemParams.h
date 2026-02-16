#ifndef SYSTEMPARAMS_H
#define SYSTEMPARAMS_H

#define RELAY_PIN 13
#define HEAD_PIN  12

struct queueBin {
  double eulerX;
  double eulerY;
  double eulerZ;
  double pidOut;
  double dX;
};

#endif