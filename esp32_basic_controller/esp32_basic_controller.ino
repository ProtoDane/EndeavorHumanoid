
// Required Libraries
#include <HardwareSerial.h>
#include <uni.h>
#include <Bluepad32.h>

#include "main.h"

// Bluepad32 instances
ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

// UART Instance
HardwareSerial servoSerial(2);

#define RELAY_PIN 13
#define HEAD_PIN  12

bool operable = true;
// Controller address string
//static const char * controller_addr_string = CTRL_MAC; 

void setup() {
  
  BP32.forgetBluetoothKeys();
  
  // Initialize Bluetooth allowlist
  bd_addr_t controller_addr;
  sscanf_bd_addr(CTRL_MAC, controller_addr);
  uni_bt_allowlist_add_addr(controller_addr);
  uni_bt_allowlist_set_enabled(true);

  //
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(HEAD_PIN, OUTPUT);
  
  // Initialize serial
  Serial.begin(115200);
  Serial.println(s_torso.mid);
  Serial.println(servoCluster[3].mid);
  servoSerial.begin(115200, SERIAL_8N1, 16, 17);

  String fv = BP32.firmwareVersion();
  Serial.print("Firmware version installed: ");
  Serial.println(fv);

  // To get the BD Address (MAC address) call:
  const uint8_t* addr = BP32.localBdAddress();
  Serial.print("BD Address: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(addr[i], HEX);
    if (i < 5)
      Serial.print(":");
    else
      Serial.println();
  }

  // Bluepad32 initialization
  BP32.setup(&onConnectedController, &onDisconnectedController);

}

// Arduino loop function. Runs in CPU 1
void loop() {

  BP32.update();

  // It is safe to always do this before using the controller API.
  // This guarantees that the controller is valid and connected.
  for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
    ControllerPtr myController = myControllers[i];

    if (myController && myController->isConnected()) {
      if (myController->isGamepad())
        mainProcess(myController);
    }
  }

  if (servoSerial.available()) {

    int input = servoSerial.read();

    if (input == 0b11110000) {
      int v_first = servoSerial.read();
      int v_last = servoSerial.read();
      int i_first = servoSerial.read();
      int i_last = servoSerial.read();

      float v = (float)(v_first) + (float)(v_last)/100.0;
      float i = (float)(i_first) + (float)(i_last)/100.0;

      Serial.println("V: " + String(v) + " I: " + String(i) );

      if (v < VOLTAGE_CUTOFF & v > 6.8) {
        // operable = false;
        // digitalWrite(RELAY_PIN, LOW);
        // cmdDisable();
      }
    }
  }

  delay(50);
}

// Top level control structure to determine action sequences based on gamepad input
void mainProcess(ControllerPtr gamepad) {

  // Relay debounce variable; provides press/release action for toggling the relay
  static bool relayState = false;

  // Check home button state and determine if a debounce action occurs and toggles the relay
  if (gamepad->miscSystem() & operable) {

    while(gamepad->miscSystem()) {BP32.update(); delay(50);}

    relayState = !relayState;
    digitalWrite(RELAY_PIN, !digitalRead(RELAY_PIN));
    digitalWrite(HEAD_PIN, !digitalRead(HEAD_PIN));
    relayState ? cmdEnable() : cmdDisable();
  }

  // Check relay state and process the remaining gamepad controls if it is high
  if (relayState & operable) {

    // Fetch left/right joystick values
    int lx = gamepad->axisX();
    int ly = gamepad->axisY();
    int rx = gamepad->axisRX();
    int ry = gamepad->axisRY();
    
    // Check buttons first, then check drive controls last.  If no inputs, send idle command to servo driver
    if (gamepad->a()) {
      
      Serial.println("[ESP32]: (B) pressed");
      actionCrouch(gamepad);

    } else if (gamepad->b()) {

      Serial.println("[ESP32]: (A) pressed");
      actionSwipe(gamepad, false);
    
    } else if (gamepad->y()) {

      Serial.println("[ESP32]: (X) pressed");
      if(!ULT_LOCK) {actionRoll(gamepad);}
    
    } else if (gamepad->x()) {

      Serial.println("[ESP32]: (Y) pressed");
      actionSwipe(gamepad, true);
    
    } else if (gamepad->dpad() & DPAD_UP) {

      Serial.println("[ESP32]: DP-UP pressed");
      actionEmote1(gamepad);
    
    } else if (gamepad->dpad() & DPAD_DOWN) {

      Serial.println("[ESP32]: DP-DOWN pressed");
      //actionEmote2(gamepad);
      actionEmote5(gamepad);

    } else if (gamepad->dpad() & DPAD_LEFT) {
    
      Serial.println("[ESP32]: DP-LEFT pressed");
      actionEmote3(gamepad);

    } else if (gamepad->dpad() & DPAD_RIGHT) {

      Serial.println("[ESP32]: DP-RIGHT pressed");
      actionEmote4(gamepad);

    } else if (gamepad->l1()) {
    
      Serial.println("[ESP32]: L-Shoulder pressed");
      actionPunch(gamepad, true);
      
    } else if (gamepad->l2()) {  
    
      Serial.println("[ESP32]: L-Trigger pressed");
      actionUpperCut(gamepad, true);
      
    } else if (gamepad->r1()) {
    
      Serial.println("[ESP32]: R-Shoulder pressed");
      actionPunch(gamepad, false);
      
    } else if (gamepad->r2()) {
    
      Serial.println("[ESP32]: R-Trigger pressed");
      actionUpperCut(gamepad, false);
      
    } else if (gamepad->thumbL()) {

      Serial.println("[ESP32]: L-Thumb pressed");

    } else if (gamepad->thumbR()) {

      Serial.println("[ESP32]: R-Thumb pressed");

    } else if (gamepad->miscBack()) {

      Serial.println("[ESP32]: (-) pressed");
      actionGetupBack(gamepad);

    } else if (gamepad->miscHome()) {

      Serial.println("[ESP32]: (+) pressed");
      actionGetupFront(gamepad);

    } else if (ly < -AXIS_THRESHOLD || ry < -AXIS_THRESHOLD) {
    
      Serial.println("[ESP32]: Walk forward");
      actionWalkFwd(gamepad);
      
    } else if (ly > AXIS_THRESHOLD || ry > AXIS_THRESHOLD) {  
    
      Serial.println("[ESP32]: Walk backward");
      actionWalkBwd(gamepad);
      
    } else if (lx > AXIS_THRESHOLD) {
    
      Serial.println("[ESP32]: Spin right");
      actionTurn(gamepad, false);
      
    } else if (lx < -AXIS_THRESHOLD) {

      Serial.println("[ESP32]: Spin left");
      actionTurn(gamepad, true);
  
    } else if (rx < -AXIS_THRESHOLD) {

      Serial.println("[ESP32]: Strafe left");
      actionStrafe(gamepad, true);
      
    } else if (rx > AXIS_THRESHOLD) {

      Serial.println("[ESP32]: Strafe right");
      actionStrafe(gamepad, false);
      
    } else {
      
      Serial.println("[ESP32]: Sent idle command");
      actionIdle();

    }
  }
}

void actionEmote5(ControllerPtr gamepad) {

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

  sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
  setServoDelay(emote5_init_1, ALL_SERVOS, 250);

  int i = 0;
  bool select = true;
  while(gamepad->dpad() & DPAD_DOWN) {

    sendCommand(RETURN_NONE, CMD_PULSE);
    if (select) {
      setServoSequence(i, emote5_1, 0b11111111100010, 480);
    } else {
      setServoSequence(i, emote5_2, 0b1111111111000, 480);
    }

    if (i == 48) {
      i = 0;
      select = !select;
      delayMicroseconds(75758);
      sendCommand(RETURN_NONE, CMD_PULSE);
      setServoCluster(select ? emote5_init_1:emote5_init_2, ALL_SERVOS);
    }
    
    i++;
    delayMicroseconds(75758);
    BP32.update();    
  }
}

void actionEmote4(ControllerPtr gamepad) {

  while(gamepad->dpad() & DPAD_RIGHT) {
    
    sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
    setServoDelay(emote4_1, RIGHT_ARM, 250);

    sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
    setServoDelay(emote4_2, RIGHT_ARM, 250);
    
    BP32.update();
  }
}

void actionEmote3 (ControllerPtr gamepad) {

  sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
  setServoDelay(emote3, ALL_SERVOS, 500);

  while (gamepad->dpad() & DPAD_LEFT) {BP32.update(); delay(50);}
}

void actionEmote2(ControllerPtr gamepad) {

  float emoteStart[] = {-20.0, -90.0, 20.0, 90.0};
  sendCommand(RETURN_NONE, CMD_PULSE);
  setServoCluster(emoteStart, 0b110000000000110);
  delay(250);

  int i = 0;
  while(gamepad->dpad() & DPAD_DOWN) {

    sendCommand(RETURN_NONE, CMD_PULSE);
    setServoSequence(i, sequence_emote2, 0b1101111111101101, 624);

    i++;
    BP32.update();
    delay(64);
  }
}

// Shikanoko nokoko koshitantan dance emote attempt
void actionEmote1(ControllerPtr gamepad) {
  int i = 3;
  while(gamepad->dpad() & DPAD_UP) {

    sendCommand(RETURN_NONE, CMD_PULSE);
    setServoSequence(i, sequence_emote1, 0b101111111100100, 120);

    i++;
    BP32.update();
    delay(54);
  }
}

void actionRoll(ControllerPtr gamepad) {

  sendCommand(RETURN_NONE, CMD_PULSE);
  setServoCluster(roll_1, ALL_SERVOS);

  delay(600);

  sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
  setServoDelay(roll_2, 0b11111111100010, 400);

  sendCommand(RETURN_NONE, CMD_PULSE);
  setServoCluster(roll_3, LEG_SERVOS);

  while (gamepad-> y()) {BP32.update(); delay(50);}
}

void actionSwipe(ControllerPtr gamepad, bool left) {

  sendCommand(RETURN_NONE, CMD_PULSE);
  setServoCluster(left ? swipeL_1 : swipeR_1, ALL_SERVOS);

  delay(250);

  sendCommand(RETURN_NONE, CMD_PULSE);
  setServoCluster(left ? swipeL_2 : swipeR_2, left ? 0b11111 : 0b11110000000000001);

  while (left & gamepad->x() || !left & gamepad->b()) {BP32.update(); delay(50);}
}

void actionUpperCut(ControllerPtr gamepad, bool left) {
  
  sendCommand(RETURN_NONE, CMD_PULSE);
  setServoCluster(left ? uppCutL_1 : uppCutR_1, ALL_SERVOS);

  delay(500);

  sendCommand(RETURN_NONE, CMD_PULSE);
  setServoCluster(uppCut_2, left ? 0b10100000000000000 : 0b10100);

  while (left & gamepad->l2() || !left & gamepad->r2() ) {BP32.update(); delay(50);}
}

void actionPunch(ControllerPtr gamepad, bool left) {

  sendCommand(RETURN_NONE, CMD_PULSE);

  setServoCluster(left ? punchL : punchR, ALL_SERVOS);

  while (left & gamepad->l1() || !left & gamepad->r1()) {BP32.update(); delay(50);}
  
}

void actionGetupFront(ControllerPtr gamepad) {
  sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
  setServoDelay(getupFront_1, ALL_SERVOS, 100);

  delay(500);

  sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
  setServoDelay(getupFront_2, 0b10000000000010000, 100);

  delay(250);
  
  sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
  setServoDelay(getupFront_3, 0b101, 100);

  while (gamepad->miscHome()) {BP32.update(); delay(50);}

  sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
  setServoDelay(idleAngles, ALL_SERVOS, 500);
}

void actionGetupBack(ControllerPtr gamepad) {
  sendCommand(RETURN_NONE, CMD_PULSE);
  setServoCluster(getupBack_1, ALL_SERVOS);

  delay(500);

  sendCommand(RETURN_NONE, CMD_PULSE);
  setServoCluster(getupBack_2, 0b10010000000010010);

  while (gamepad->miscBack()) {BP32.update(); delay(50);}
}

void actionStrafe(ControllerPtr gamepad, bool left) {
  int i = 0;
  while ( (left & gamepad->axisRX() < -AXIS_THRESHOLD) || (!left & gamepad->axisRX() > AXIS_THRESHOLD) ) {

    sendCommand(RETURN_NONE, CMD_PULSE);

    if (left) {
      setServoSequence(i, sequence_strafeL, 0b1111111100000, sizeof(sequence_strafeL) / sizeof(sequence_strafeL[0]));
    } else {
      setServoSequence(i, sequence_strafeR, 0b1111111100000, sizeof(sequence_strafeR) / sizeof(sequence_strafeR[0]));
    }

    i++;
    BP32.update();
    delay(40);
  }
}

void actionTurn(ControllerPtr gamepad, bool ccw) {

  for (int i = 0; i < 3; i++) {
    sequence_turn[9 * i] = ccw ? -15.0 : 15.0;
  }

  for (int i = 3; i < 6; i++) {
    sequence_turn[9 * i] = ccw ? 15.0 : -15.0;
  }

  int i = 0;
  while ( (ccw & gamepad->axisX() < -AXIS_THRESHOLD) || (!ccw & gamepad->axisX() > AXIS_THRESHOLD) ) {

    sendCommand(RETURN_NONE, CMD_PULSE);
    setServoSequence(i, sequence_turn, 0b1111111100001, sizeof(sequence_turn) / sizeof(sequence_turn[0]));

    i++;
    BP32.update();
    delay(30);
  }
}

void actionWalkBwd(ControllerPtr gamepad) {

  sendCommand(RETURN_NONE, CMD_PULSE);
  setServoSequence(0 ,sequence_initBwd, 0b00011111111100011, sizeof(sequence_initBwd) / sizeof(sequence_initBwd[0]));
  delay(200);

  int i = 1;
  while ( (gamepad->axisY() > AXIS_THRESHOLD || gamepad->axisRY() > AXIS_THRESHOLD) & i < 6) {

    sendCommand(RETURN_NONE, CMD_PULSE);
    setServoSequence(i ,sequence_initBwd, 0b00011111111100011, sizeof(sequence_initBwd) / sizeof(sequence_initBwd[0]));

    i++;
    BP32.update();
    delay(25);
  }

  i = 3;
  while (gamepad->axisY() > AXIS_THRESHOLD || gamepad->axisRY() > AXIS_THRESHOLD) {
    
    sendCommand(RETURN_NONE, CMD_PULSE);
    setServoSequence(i, sequence_walkBwd, 0b00011111111100011, sizeof(sequence_walkBwd) / sizeof(sequence_walkBwd[0]));
    
    i++;
    BP32.update();
    delay(45);
  }
}

void actionWalkFwd(ControllerPtr gamepad) {
  
  sendCommand(RETURN_NONE, CMD_PULSE);
  setServoSequence(0 ,sequence_initFwd, 0b00011111111100011, sizeof(sequence_initFwd) / sizeof(sequence_initFwd[0]));
  delay(200);

  int i = 1;
  while ( (gamepad->axisY() < -AXIS_THRESHOLD || gamepad->axisRY() < -AXIS_THRESHOLD) & i < 6) {

    sendCommand(RETURN_NONE, CMD_PULSE);
    setServoSequence(i ,sequence_initFwd, 0b00011111111100011, sizeof(sequence_initFwd) / sizeof(sequence_initFwd[0]));

    i++;
    BP32.update();
    delay(25);
  }

  i = 3;
  while (gamepad->axisY() < -AXIS_THRESHOLD || gamepad->axisRY() < -AXIS_THRESHOLD) {
    
    sendCommand(RETURN_NONE, CMD_PULSE);
    setServoSequence(i, sequence_walkFwd, 0b00011111111100011, sizeof(sequence_walkFwd) / sizeof(sequence_walkFwd[0]));
    
    i++;
    BP32.update();
    delay(45);
  }
}

void actionCrouch(ControllerPtr gamepad) {

  sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
  setServoDelay(crouchAngles, ALL_SERVOS, 150);
  //setServoCluster(crouchAngles, ALL_SERVOS);

  while(gamepad->a()) {BP32.update(); delay(50);}

}

void actionIdle() {
  
  sendCommand(RETURN_VA, CMD_PULSE);
  setServoCluster(idleAngles, ALL_SERVOS);
}

void cmdEnable() {

  Serial.println("Enabling Servo2040...");
  sendCommand(RETURN_NONE, CMD_ENABLE);
  setServoCluster(idleAngles, ALL_SERVOS); //0b11110000000011111

  

  delay(250);

}

void cmdDisable() {

  Serial.println("Disabling Servo2040...");
  sendCommand(RETURN_NONE, CMD_DISABLE);
}

void sendCommand(int preamble, int cmd) {
  servoSerial.write(preamble << 4 | cmd);
}

void setServoDelay(float angles[], int pinMask, int delayMs) {
  int first = delayMs / 100;
  int second = delayMs % 100;
  servoSerial.write(first);
  servoSerial.write(second);

  setServoCluster(angles, pinMask);
  delay(delayMs);
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

void setServo(int pulse) {
  int first = pulse / 100;
  int second = pulse % 100;
  servoSerial.write(first);
  servoSerial.write(second);
}

//
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

// Handler for controller connected event
void onConnectedController(ControllerPtr ctl) {
  
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    
    if (myControllers[i] == nullptr) {
      
      myControllers[i] = ctl;
      foundEmptySlot = true;

      // Optional, once the gamepad is connected, request further info about the
      // gamepad.
      ControllerProperties properties = ctl->getProperties();
      char buf[80];
      sprintf(buf,
              "BTAddr: %02x:%02x:%02x:%02x:%02x:%02x, VID/PID: %04x:%04x, "
              "flags: 0x%02x",
              properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
              properties.btaddr[3], properties.btaddr[4], properties.btaddr[5],
              properties.vendor_id, properties.product_id, properties.flags);
      Serial.println(buf);
      break;
    }
  }
  
  if (!foundEmptySlot) {
    
    Serial.println(
        "CALLBACK: Controller connected, but could not found empty slot");
  }
}

// Handler for controller disconnected event
void onDisconnectedController(ControllerPtr ctl) {
  bool foundGamepad = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    
    if (myControllers[i] == ctl) {
      
      Serial.print("CALLBACK: Controller is disconnected from index=");
      Serial.println(i);
      myControllers[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }

  if (!foundGamepad) {
    
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}
