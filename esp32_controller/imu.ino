void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void process_bno08x() {
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 10 * 1000;  
  sh2_SensorValue_t sensorValue;
  double filteredPitch = 0.0;

  setReports(reportType, reportIntervalUs);
  delay(100);

  for(;;) {
    if (bno08x.wasReset()) {
      Serial.print("sensor was reset ");
      setReports(reportType, reportIntervalUs);
    }
    
    if (bno08x.getSensorEvent(&sensorValue)) {
      // in this demo only one report type will be received depending on FAST_MODE define (above)
      switch (sensorValue.sensorId) {
        case SH2_ARVR_STABILIZED_RV:
          quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        case SH2_GYRO_INTEGRATED_RV:
          // faster (more noise?)
          quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
          break;
      }

      queueBin msg;

      filteredPitch = 0.5 * ypr.pitch + (1 - 0.5) * filteredPitch;
      pidIn = -1 * filteredPitch;
      pidSet = 0;
      pidPitch.Compute();
      msg.eulerY = -1 * filteredPitch;
      msg.pidOut = pidOut;
      msg.dX = 125.0 * tan(radians(pidOut));

      xQueueOverwrite(imuQueue, (void *) &msg);          

      if (tipSafetyEnabled && abs(filteredPitch) > IMU_TIP_THRESHOLD && FALL_PROTECTION_ENABLED) {
        // Serial.printf("TIP DETECTED, SHUTTING OFF SERVOS (PITCH = %lf)\n", msg.eulerY);
        relayState = false;
        tipSafetyEnabled = false;

        if (!SERIAL_DEBUG_MODE) {
          digitalWrite(RELAY_PIN, LOW);
          digitalWrite(HEAD_PIN, LOW);
        }
      }
    }
  }
}

void process_bno055() {

  double filteredPitch = 0.0;

  for(;;) {

    if (servoSerial.available()) {
      int input = servoSerial.read();

      if (input == 0b11110000) {
        
        // Do something with V/I inputs (unused)

      } else if (input == 0b10010000) {

        while (servoSerial.available() < 24);

        uint8_t buffer[24];
        servoSerial.readBytes(buffer, 24);

        queueBin msg;
        memcpy(&msg.eulerX, &buffer[0], 8);
        memcpy(&msg.eulerY, &buffer[8], 8);
        memcpy(&msg.eulerZ, &buffer[16], 8);

        filteredPitch = 0.5 * msg.eulerY + (1 - 0.5) * filteredPitch;
        pidIn = filteredPitch;
        pidSet = 0;
        pidPitch.Compute();
        msg.pidOut = pidOut;
        msg.dX = 125.0 * tan(radians(pidOut));

        xQueueOverwrite(imuQueue, (void *) &msg);          

        if (tipSafetyEnabled && abs(msg.eulerY) > IMU_TIP_THRESHOLD && FALL_PROTECTION_ENABLED) {
          // Serial.printf("TIP DETECTED, SHUTTING OFF SERVOS (PITCH = %lf)\n", msg.eulerY);
          relayState = false;
          tipSafetyEnabled = false;

          if (!SERIAL_DEBUG_MODE) {
            digitalWrite(RELAY_PIN, LOW);
            digitalWrite(HEAD_PIN, LOW);
          }
        }
      }
    }
    delay(10);
  }   
}


