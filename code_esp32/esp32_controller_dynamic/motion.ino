// MOTION.INO
// 
// Section of the Arduino sketch that manages the various motion sequences that the robot performs

// Idle standing sequence.  Uses PID data to shift x-axis distance in response to changes in pitch angle
void actionIdle() {

  queueBin bin;
  getIMU(&bin);

  legAngles l;
  armAngles a = {90.0, 60.0, 0.0, 20.0, -90.0, -60.0, 0.0, -20.0};
  ik_legs(&l, 20.0 - bin.dX, 8.0, 125.0, 20.0 - bin.dX, 8.0, 125.0);

  if (l.success) {
    sendCommand(RETURN_NONE, CMD_PULSE);
    setServoCluster2(&l, &a, 0.0);
  } else {
    sendCommand(RETURN_NONE, CMD_PULSE);
    setServoCluster(idleAngles, ALL_SERVOS); 
  }
}

// Walk forward sequence
void actionWalkFwd(ControllerPtr gamepad) {
  
  // Trajectory parameters
  double z0 = 125.0, nZ = 30.0, pZ = 15.0;  // z0: initial standing height | nZ: vertical up distance | pZ: vertical down distance
  double y0 = 5.0, dY = 15.0;               // y0: initial sideways offset | dY: sideways foot amplitude
  double x0 = 10.0,  dX = 20.0;             // x0: initial front/back foot distance | dX: step amplitude
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
      sendCommand(RETURN_NONE, CMD_PULSE);
      setServoCluster2(&l, &a, dT * sin(i * PI / 12));
    } else {
      return;
    }

    BP32.update();
    delay(20);

    // Escape condition if the stick is no longer held
    if (gamepad->axisY() >= -AXIS_THRESHOLD && gamepad->axisRY() >= -AXIS_THRESHOLD) {
      return;
    }
  }

  // Continuous sequence
  int i = 0;
  while (gamepad->axisY() < -AXIS_THRESHOLD || gamepad->axisRY() < -AXIS_THRESHOLD) {
    
    queueBin bin;
    getIMU(&bin);

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
      sendCommand(RETURN_NONE, CMD_PULSE);
      setServoCluster2(&l, &a, dT * cos(i * PI / 6));
    } else {
      return;
    }

    i++;
    BP32.update();
    delay(35);
  }
}

void actionCrouch(ControllerPtr gamepad) {

  sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
  setServoDelay(crouchAngles, ALL_SERVOS, 150);
  //setServoCluster(crouchAngles, ALL_SERVOS);

  while(gamepad->a()) {BP32.update(); delay(50);}

}