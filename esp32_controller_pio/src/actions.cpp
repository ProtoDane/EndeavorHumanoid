#include "actions.h"

actionHandler::actionHandler() {
    // Constructor method
}

// ==========================================================================
// Public Functions
// ==========================================================================

void actionHandler::begin(servoHandler servo, QueueHandle_t imuQueue) {
    _servo = servo;
    _imuQueue = imuQueue;
}

void actionHandler::actionIdle() {

    queueBin bin;
    fetchQueue(&bin);

    legAngles l;
    armAngles a = {90.0, 60.0, 0.0, 20.0, -90.0, -60.0, 0.0, -20.0};

    ik_legs(&l, 20.0 - bin.dX, 8.0, 125.0, 20.0 - bin.dX, 8.0, 125.0);

    if (l.success) {
        _servo.sendCommand(RETURN_NONE, CMD_PULSE);
        _servo.setServoCluster(&l, &a, 0.0);
    } else {
        _servo.sendCommand(RETURN_NONE, CMD_PULSE);
        _servo.setServoCluster(idleAngles, ALL_SERVOS);
    }
}

// Walk forward sequence
void actionHandler::moveWalkFwd(ControllerPtr gamepad) {
  
    // Trajectory parameters
    double z0 = 125.0, nZ = 30.0, pZ = 15.0;  // z0: initial standing height | nZ: vertical up distance | pZ: vertical down distance
    double y0 = 10.0, dY = 10.0;               // y0: initial sideways offset | dY: sideways foot amplitude
    double x0 = -10.0,  dX = 20.0;             // x0: initial front/back foot distance | dX: step amplitude
    double dT = 10.0, dA = 10.0;              // dT: torso angular amplitude | dA: shoulder joint amplitude

    // Initial sequence steps
    for (int i = 0; i < 6; i++) {
        
        legAngles l;
        armAngles a = {90.0 + dA * sin(i * PI / 12), 60.0, 0.0, 20.0, -90.0 + dA * sin(i * PI / 12), -60.0, 0.0, -20.0};
        ik_legs(&l, 
        x0 + dX * i / 6, 
        y0 + dY * sin(i * PI / 6), 
        z0 - nZ * sin(i * PI / 6), 
        x0 - dX * i / 6, 
        y0 - dY * sin(i * PI / 6), 
        z0 + pZ * sin(i * PI / 6)
        );    

        // Perform sanity check to make sure IK calculation was successful
        if (l.success) {
            _servo.sendCommand(RETURN_NONE, CMD_PULSE);
            _servo.setServoCluster(&l, &a, dT * sin(i * PI / 12));
        } else {
            return;
        }

        BP32.update();
        delay(25);  // 25

        // Escape condition if the stick is no longer held
        if (gamepad->axisY() >= -AXIS_THRESHOLD && gamepad->axisRY() >= -AXIS_THRESHOLD) {
            return;
        }
    }

    // Continuous sequence
    int i = 0;
    while (gamepad->axisY() < -AXIS_THRESHOLD || gamepad->axisRY() < -AXIS_THRESHOLD) {
        
        queueBin bin;
        fetchQueue(&bin);

        legAngles l;
        armAngles a = {90.0 + dA * cos(i * PI / 6), 60.0, 0.0, 20.0, -90.0 + dA * cos(i * PI / 6), -60.0, 0.0, -20.0};
        ik_legs(&l, 
        (i%12 < 6) ? (x0 - bin.dX + dX * (1 - (i%6)/3)):(x0 - bin.dX + dX * ((i%6)/3 - 1)), 
        y0 - dY * sin(PI * i / 6), 
        (i%12 <= 6) ? (z0 + pZ * sin(PI * (i%6) / 6)):(z0 - nZ * sin(PI * (i%6) / 6)), 
        (i%12 <= 6) ? (x0 - bin.dX + dX * ((i%6)/3 - 1)):(x0 - bin.dX + dX * (1 - (i%6)/3)), 
        y0 + dY * sin(PI * i / 6), 
        (i%12 < 6) ? (z0 - nZ * sin(PI * (i%6) / 6)):(z0 + pZ * sin(PI * (i%6) / 6))
        );    

        // Perform sanity check to make sure IK calculation was successful
        if (l.success) {
            _servo.sendCommand(RETURN_NONE, CMD_PULSE);
            _servo.setServoCluster(&l, &a, dT * cos(i * PI / 6));
        } else {
            return;
        }

        i++;
        BP32.update();
        delay(35); // 30
    }
}

void actionHandler::moveWalkBwd(ControllerPtr gamepad) {
    
    // Trajectory parameters
    double z0 = 125.0, nZ = 30.0, pZ = 15.0;  // z0: initial standing height | nZ: vertical up distance | pZ: vertical down distance
    double y0 = 10.0, dY = 10.0;               // y0: initial sideways offset | dY: sideways foot amplitude
    double x0 = 20.0,  dX = 20.0;             // x0: initial front/back foot distance | dX: step amplitude
    double dT = 10.0, dA = 10.0;              // dT: torso angular amplitude | dA: shoulder joint amplitude

    // Initial sequence steps
    for (int i = 0; i < 6; i++) {
        
        legAngles l;
        armAngles a = {90.0 + dA * sin(i * PI / 12), 60.0, 0.0, 20.0, -90.0 + dA * sin(i * PI / 12), -60.0, 0.0, -20.0};
        ik_legs(&l, 
            x0 + dX * i / 6, 
            y0 - dY * sin(i * PI / 6), 
            z0 + pZ * sin(i * PI / 6), 
            x0 - dX * i / 6, 
            y0 + dY * sin(i * PI / 6), 
            z0 - nZ * sin(i * PI / 6)
        );    

        // Perform sanity check to make sure IK calculation was successful
        if (l.success) {
            _servo.sendCommand(RETURN_NONE, CMD_PULSE);
            _servo.setServoCluster(&l, &a, dT * sin(i * PI / 12));
        } else {
            return;
        }

        BP32.update();
        delay(20);

        // Escape condition if the stick is no longer held
        if (gamepad->axisY() < AXIS_THRESHOLD && gamepad->axisRY() < AXIS_THRESHOLD) {
            return;
        }
    }

    // Continuous sequence
    int i = 0;
    while (gamepad->axisY() > AXIS_THRESHOLD || gamepad->axisRY() > AXIS_THRESHOLD) {
        
        queueBin bin;
        fetchQueue(&bin);
        

        legAngles l;
        armAngles a = {90.0 + dA * cos(i * PI / 6), 60.0, 0.0, 20.0, -90.0 + dA * cos(i * PI / 6), -60.0, 0.0, -20.0};
        ik_legs(&l, 
            (i%12 < 6) ? (x0 - bin.dX + dX * (1 - (i%6)/3)):(x0 - bin.dX + dX * ((i%6)/3 - 1)), 
            y0 + dY * sin(PI * i / 6), 
            (i%12 <= 6) ? (z0 - nZ * sin(PI * (i%6) / 6)):(z0 + pZ * sin(PI * (i%6) / 6)), 
            (i%12 <= 6) ? (x0 - bin.dX + dX * ((i%6)/3 - 1)):(x0 - bin.dX + dX * (1 - (i%6)/3)), 
            y0 - dY * sin(PI * i / 6), 
            (i%12 < 6) ? (z0 + pZ * sin(PI * (i%6) / 6)):(z0 - nZ * sin(PI * (i%6) / 6))
        );    

        // Perform sanity check to make sure IK calculation was successful
        if (l.success) {
            _servo.sendCommand(RETURN_NONE, CMD_PULSE);
            _servo.setServoCluster(&l, &a, dT * cos(i * PI / 6));
        } else {
            return;
        }

        i++;
        BP32.update();
        delay(35);
    }
}

void actionHandler::moveStrafe(ControllerPtr gamepad, bool dir) {
    int i = 0;
    while ( (dir & gamepad->axisRX() < -AXIS_THRESHOLD) || (!dir & gamepad->axisRX() > AXIS_THRESHOLD) ) {

        _servo.sendCommand(RETURN_NONE, CMD_PULSE);

        if (dir) {
            _servo.setServoSequence(i, sequence_strafeL, 0b1111111100000, sizeof(sequence_strafeL) / sizeof(sequence_strafeL[0]));
        } else {
            _servo.setServoSequence(i, sequence_strafeR, 0b1111111100000, sizeof(sequence_strafeR) / sizeof(sequence_strafeR[0]));
        }

        i++;
        BP32.update();
        delay(40);
    }
}

void actionHandler::moveSpin(ControllerPtr gamepad, bool dir) {
    for (int i = 0; i < 3; i++) {
        sequence_turn[9 * i] = dir ? -15.0 : 15.0;
    }

    for (int i = 3; i < 6; i++) {
        sequence_turn[9 * i] = dir ? 15.0 : -15.0;
    }

    int i = 0;
    while ( (dir & gamepad->axisX() < -AXIS_THRESHOLD) || (!dir & gamepad->axisX() > AXIS_THRESHOLD) ) {

        _servo.sendCommand(RETURN_NONE, CMD_PULSE);
        _servo.setServoSequence(i, sequence_turn, 0b1111111100001, sizeof(sequence_turn) / sizeof(sequence_turn[0]));

        i++;
        BP32.update();
        delay(30);
    }
}

void actionHandler::actionGetup(ControllerPtr gamepad) {
    _servo.sendCommand(RETURN_NONE, CMD_NONE);
    queueBin q;
    fetchQueue(&q);
    
    double pitch = q.eulerY;
    // Serial.print("PITCH: " + String(pitch) + " | ");
    if (pitch > 30.0) {
        // Serial.println("Robot facing up!");
        _servo.sendCommand(RETURN_NONE, CMD_PULSE);
        _servo.setServoCluster(getupBack_1, ALL_SERVOS);

        delay(500);

        _servo.sendCommand(RETURN_NONE, CMD_PULSE);
        _servo.setServoCluster(getupBack_2, 0b10010000000010010);

    } else if (pitch < -30.0) {
        // Serial.println("Robot facing down!");
        _servo.sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
        _servo.setServoDelay(getupFront_1, ALL_SERVOS, 100);

        delay(500);

        _servo.sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
        _servo.setServoDelay(getupFront_2, 0b10000000000010000, 100);

        delay(250);
        
        _servo.sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
        _servo.setServoDelay(getupFront_3, 0b101, 100);
    } else {
        // Serial.println("Robot not tipped..");
    }

    while (gamepad->miscHome()) {BP32.update(); delay(50);}
    _servo.sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
    _servo.setServoDelay(idleAngles, ALL_SERVOS, 500);    
}

void actionHandler::actionPunch(ControllerPtr gamepad, bool dir) {
    _servo.sendCommand(RETURN_NONE, CMD_PULSE);

    _servo.setServoCluster(dir ? punchL : punchR, ALL_SERVOS);

    while (dir & gamepad->l1() || !dir & gamepad->r1()) {BP32.update(); delay(50);}
}

void actionHandler::actionUpperCut(ControllerPtr gamepad, bool dir) {
    _servo.sendCommand(RETURN_NONE, CMD_PULSE);
    _servo.setServoCluster(dir ? uppCutL_1 : uppCutR_1, ALL_SERVOS);

    delay(500);

    _servo.sendCommand(RETURN_NONE, CMD_PULSE);
    _servo.setServoCluster(uppCut_2, dir ? 0b10100000000000000 : 0b10100);

    while (dir & gamepad->l2() || !dir & gamepad->r2() ) {BP32.update(); delay(50);}
}

void actionHandler::actionSwipe(ControllerPtr gamepad, bool dir) {
    _servo.sendCommand(RETURN_NONE, CMD_PULSE);
    _servo.setServoCluster(dir ? swipeL_1 : swipeR_1, ALL_SERVOS);

    delay(250);

    _servo.sendCommand(RETURN_NONE, CMD_PULSE);
    _servo.setServoCluster(dir ? swipeL_2 : swipeR_2, dir ? 0b11111 : 0b11110000000000001);

    while (dir & gamepad->x() || !dir & gamepad->b()) {BP32.update(); delay(50);}
}

void actionHandler::actionTumble(ControllerPtr gamepad) {
    _servo.sendCommand(RETURN_NONE, CMD_PULSE);
    _servo.setServoCluster(roll_1, ALL_SERVOS);

    delay(600);

    _servo.sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
    _servo.setServoDelay(roll_2, 0b11111111100010, 400);

    _servo.sendCommand(RETURN_NONE, CMD_PULSE);
    _servo.setServoCluster(roll_3, LEG_SERVOS);

    while (gamepad-> y()) {BP32.update(); delay(50);}
}

void actionHandler::emoteShikanoko(ControllerPtr gamepad) {
    int i = 3;
    while(gamepad->dpad() & DPAD_LEFT) {

        _servo.sendCommand(RETURN_NONE, CMD_PULSE);
        _servo.setServoSequence(i, sequence_emote1, 0b101111111100100, 120);

        i++;
        BP32.update();
        delay(54);
    }
}

void actionHandler::emoteHero(ControllerPtr gamepad) {

    _servo.sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
    _servo.setServoDelay(emote3, ALL_SERVOS, 500);

    while (gamepad->dpad() & DPAD_UP) {BP32.update(); delay(50);}  
}

void actionHandler::emoteTaunt(ControllerPtr gamepad) {
    while(gamepad->dpad() & DPAD_RIGHT) {
        
        _servo.sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
        _servo.setServoDelay(emote4_1, RIGHT_ARM, 250);

        _servo.sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
        _servo.setServoDelay(emote4_2, RIGHT_ARM, 250);
        
        BP32.update();
    }
}

void actionHandler::emoteGangnam(ControllerPtr gamepad) {
  float emote5_init_1[] = {
    0.0, 
    0.0, -90.0, -90.0, 45.0, 
    -9.1, -46.6, 33.1, -9.1,
    9.1, 46.6, -33.1, 9.1, 
    0.0, 90.0, 90.0, -45.0
  };

  float emote5_init_2[] = {
    0.0, 
    0.0, -90.0, 0.0, 0.0, 
    -9.1, -46.6, 33.1, -9.1,
    9.1, 46.6, -33.1, 9.1, 
    0.0, 90.0, 90.0, 0.0    
  };

  _servo.sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
  _servo.setServoDelay(emote5_init_1, ALL_SERVOS, 250);

  int i = 0;
  bool select = true;
  while(gamepad->dpad() & DPAD_DOWN) {

    _servo.sendCommand(RETURN_NONE, CMD_PULSE);
    if (select) {
      _servo.setServoSequence(i, emote5_1, 0b11111111100010, 480);
    } else {
      _servo.setServoSequence(i, emote5_2, 0b1111111111000, 480);
    }

    if (i == 48) {
      i = 0;
      select = !select;
      delayMicroseconds(75758);
      _servo.sendCommand(RETURN_NONE, CMD_PULSE);
      _servo.setServoCluster(select ? emote5_init_1:emote5_init_2, ALL_SERVOS);
    }
    
    i++;
    delayMicroseconds(75758);
    BP32.update();    
  }
}

void actionHandler::actionCrouch(ControllerPtr gamepad) {
    _servo.sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
    _servo.setServoDelay(crouchAngles, ALL_SERVOS, 150);
    //setServoCluster(crouchAngles, ALL_SERVOS);

    while(gamepad->a()) {BP32.update(); delay(50);}
}

// ==========================================================================
// Private Functions
// ==========================================================================

void actionHandler::fetchQueue(queueBin *q) {
    if (uxQueueMessagesWaiting(_imuQueue) > 0) {
        xQueuePeek(_imuQueue, q, 0);
    }
}