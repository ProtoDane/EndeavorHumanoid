
// Required Libraries
#include <uni.h>
#include <Bluepad32.h>
#include <PID_v1.h>
#include <Adafruit_BNO08x.h>

// Header files
#include "config.h"
#include "kinematics.h"
#include "servo2040.h"
#include "movesets.h"

// Bluepad32 instances
ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

// BNO08x Setup (if being used)
HardwareSerial IMUSerial(1);
Adafruit_BNO08x bno08x(-1);

// GPIO Pins
#define RELAY_PIN 13
#define HEAD_PIN  12

// Control variables
bool operable = true;
bool enableBalancing = false;
volatile bool relayState = false;
volatile bool tipSafetyEnabled = false;

// PID
double Kp = 1.0; // go lower
double Ki = 0.0;
double Kd = 0.02;
double pidSet, pidIn, pidOut;
PID pidPitch(&pidIn, &pidOut, &pidSet, Kp, Ki, Kd, DIRECT);

// Dual core instances
TaskHandle_t h_taskSerialIn;
QueueHandle_t imuQueue;

struct queueBin {
  double eulerX;
  double eulerY;
  double eulerZ;
  double pidOut;
  double dX;
};

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

void getIMU(queueBin *q) {
  if (uxQueueMessagesWaiting(imuQueue) > 0) {
    xQueuePeek(imuQueue, q, 0);
  }
}

// Core1 task to process incoming UART messages from the Servo2040 (mainly IMU data)
void taskSerialIn(void *pvParameters) {
  
  pidPitch.SetMode(AUTOMATIC);
  pidPitch.SetOutputLimits(-20, 20);
  pidPitch.SetSampleTime(10);

  delay(5000);
  imuQueue = xQueueCreate(1, sizeof(struct queueBin));

  switch(IMU_CONFIG) {
    case IMU_DISABLED:
      Serial.println("No IMU selected");
      break;
    case IMU_055:
      Serial.println("BNO-055 Selected");
      process_bno055();
      break;
    case IMU_08X:
      Serial.println("BNO-08X Selected");
      process_bno08x();
      break;
  }
  while(1) {delay(10);}
}
  

void setup() {
  
  // Initialize GPIO
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(HEAD_PIN, OUTPUT);
  
  // Initialize serial
  Serial.begin(115200);
  servoSerial.begin(115200, SERIAL_8N1, 16, 17);

  xTaskCreatePinnedToCore(taskSerialIn, "taskSerialIn", 10000, NULL, 1, &h_taskSerialIn, 1);
  delay(500);

  BP32.forgetBluetoothKeys();
  
  // Initialize Bluetooth allowlist
  bd_addr_t controller_addr;
  sscanf_bd_addr(CTRL_MAC, controller_addr);
  uni_bt_allowlist_add_addr(controller_addr);
  uni_bt_allowlist_set_enabled(true);

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

  if (SERIAL_DEBUG_MODE) {
    Serial.println("Serial debug mode!");
  } else {
    Serial.println("SERIAL DEBUG MODE DISABLED, DO NOT ACTIVATE SERVOS");
  }

  if (IMU_CONFIG == IMU_08X) {
    IMUSerial.begin(115200, SERIAL_8N1, 18, 19);
    if (!bno08x.begin_UART(&IMUSerial)) {
      Serial.println("Failed to find BNO08x chip");
      while(1) {delay(10);}
    }
  }

  // Send handshake code to Servo2040
  servoSerial.write(0b01101001);
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

  delay(10);
}

// Top level control structure to determine action sequences based on gamepad input
void mainProcess(ControllerPtr gamepad) {

  // Relay debounce variable; provides press/release action for toggling the relay


  // Check home button state and determine if a debounce action occurs and toggles the relay
  if (gamepad->miscSystem() & operable) {

    while(gamepad->miscSystem()) {BP32.update(); delay(50);}

    relayState = !relayState;

    if (!SERIAL_DEBUG_MODE) {
      digitalWrite(RELAY_PIN, !digitalRead(RELAY_PIN));
      digitalWrite(HEAD_PIN, !digitalRead(HEAD_PIN));
    }

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
      
      // Serial.println("[ESP32]: (B) pressed");
      tipSafetyEnabled = true;
      actionCrouch(gamepad);

    } else if (gamepad->b()) {

      // Serial.println("[ESP32]: (A) pressed");
      tipSafetyEnabled = true;
      actionSwipe(gamepad, false);
    
    } else if (gamepad->y()) {

      // Serial.println("[ESP32]: (X) pressed");
      if(!ULT_LOCK) {
        tipSafetyEnabled = true;
        actionRoll(gamepad);
      }
    
    } else if (gamepad->x()) {

      // Serial.println("[ESP32]: (Y) pressed");
      tipSafetyEnabled = true;
      actionSwipe(gamepad, true);
    
    } else if (gamepad->dpad() & DPAD_UP) {

      // Serial.println("[ESP32]: DP-UP pressed");
      tipSafetyEnabled = true;
      actionEmote1(gamepad);
    
    } else if (gamepad->dpad() & DPAD_DOWN) {

      // Serial.println("[ESP32]: DP-DOWN pressed");
      //actionEmote2(gamepad);
      tipSafetyEnabled = true;
      actionEmote5(gamepad);

    } else if (gamepad->dpad() & DPAD_LEFT) {
    
      // Serial.println("[ESP32]: DP-LEFT pressed");
      tipSafetyEnabled = true;
      actionEmote3(gamepad);

    } else if (gamepad->dpad() & DPAD_RIGHT) {

      // Serial.println("[ESP32]: DP-RIGHT pressed");
      tipSafetyEnabled = true;
      actionEmote4(gamepad);

    } else if (gamepad->l1()) {
    
      // Serial.println("[ESP32]: L-Shoulder pressed");
      tipSafetyEnabled = true;
      actionPunch(gamepad, true);
      
    } else if (gamepad->l2()) {  
    
      // Serial.println("[ESP32]: L-Trigger pressed");
      tipSafetyEnabled = true;
      actionUpperCut(gamepad, true);
      
    } else if (gamepad->r1()) {
    
      // Serial.println("[ESP32]: R-Shoulder pressed");
      tipSafetyEnabled = true;
      actionPunch(gamepad, false);
      
    } else if (gamepad->r2()) {
    
      // Serial.println("[ESP32]: R-Trigger pressed");
      tipSafetyEnabled = true;
      actionUpperCut(gamepad, false);
      
    // } else if (gamepad->thumbL()) {
    // } else if (gamepad->thumbR()) {
    // } else if (gamepad->miscBack()) {   // (-) Button Pressed

    } else if (gamepad->miscHome()) {   // (+) Button Pressed

      // actionGetupFront(gamepad);
      fallRecovery(gamepad);
      tipSafetyEnabled = true;

    } else if (ly < -AXIS_THRESHOLD || ry < -AXIS_THRESHOLD) {
    
      // Serial.println("[ESP32]: Walk forward");
      tipSafetyEnabled = true;
      actionWalkFwd(gamepad);
      
    } else if (ly > AXIS_THRESHOLD || ry > AXIS_THRESHOLD) {  
    
      // Serial.println("[ESP32]: Walk backward");
      tipSafetyEnabled = true;
      actionWalkBwd(gamepad);
      
    } else if (lx > AXIS_THRESHOLD) {
    
      // Serial.println("[ESP32]: Spin right");
      tipSafetyEnabled = true;
      actionTurn(gamepad, false);
      
    } else if (lx < -AXIS_THRESHOLD) {

      // Serial.println("[ESP32]: Spin left");
      tipSafetyEnabled = true;
      actionTurn(gamepad, true);
  
    } else if (rx < -AXIS_THRESHOLD) {

      // Serial.println("[ESP32]: Strafe left");
      tipSafetyEnabled = true;
      actionStrafe(gamepad, true);
      
    } else if (rx > AXIS_THRESHOLD) {

      // Serial.println("[ESP32]: Strafe right");
      tipSafetyEnabled = true;
      actionStrafe(gamepad, false);
      
    } else {

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

void fallRecovery(ControllerPtr gamepad) {
  sendCommand(RETURN_NONE, CMD_NONE);
  queueBin q;
  getIMU(&q);
  double pitch = q.eulerY;
  // Serial.print("PITCH: " + String(pitch) + " | ");
  if (pitch > 30.0) {
    // Serial.println("Robot facing up!");
    sendCommand(RETURN_NONE, CMD_PULSE);
    setServoCluster(getupBack_1, ALL_SERVOS);

    delay(500);

    sendCommand(RETURN_NONE, CMD_PULSE);
    setServoCluster(getupBack_2, 0b10010000000010010);

  } else if (pitch < -30.0) {
    // Serial.println("Robot facing down!");
    sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
    setServoDelay(getupFront_1, ALL_SERVOS, 100);

    delay(500);

    sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
    setServoDelay(getupFront_2, 0b10000000000010000, 100);

    delay(250);
    
    sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
    setServoDelay(getupFront_3, 0b101, 100);
  } else {
    // Serial.println("Robot not tipped..");
  }

  while (gamepad->miscHome()) {BP32.update(); delay(50);}
  sendCommand(RETURN_NONE, CMD_PULSE_DELAY);
  setServoDelay(idleAngles, ALL_SERVOS, 500);
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
      // Serial.println(buf);
      break;
    }
  }
  
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

// Handler for controller disconnected event
void onDisconnectedController(ControllerPtr ctl) {
  bool foundGamepad = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    
    if (myControllers[i] == ctl) {
      // Serial.print("CALLBACK: Controller is disconnected from index=");
      // Serial.println(i);
      myControllers[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }

  if (!foundGamepad) {
    // Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}
