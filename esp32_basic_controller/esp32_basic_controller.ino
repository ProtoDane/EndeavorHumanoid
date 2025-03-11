
// Required Libraries
#include <HardwareSerial.h>
#include <uni.h>
#include <Bluepad32.h>

#include "main.h"

// Bluepad32 instances
ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

// UART Instance
HardwareSerial servoSerial(2);

// Controller address string
// static const char * controller_addr_string = "98:b6:7f:0a:fa:59";
static const char * controller_addr_string = "98:b6:d5:20:65:22"; 

void setup() {
  
  BP32.forgetBluetoothKeys();
  // Initialize Bluetooth allowlist
  bd_addr_t controller_addr;
  sscanf_bd_addr(controller_addr_string, controller_addr);

  uni_bt_allowlist_add_addr(controller_addr);
  uni_bt_allowlist_set_enabled(true);

  //delay(3000);
  
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
  /*
  if (servoSerial.available()) {

    String servoBuffer = servoSerial.readStringUntil('\n');
    Serial.print("[S2040]: ");
    Serial.println(servoBuffer);
  }
  */
  delay(50);
}

// Top level control structure to determine action sequences based on gamepad input
void mainProcess(ControllerPtr gamepad) {

  // Relay debounce variable; provides press/release action for toggling the relay
  static bool relayDebounce = false;
  static bool relayState = false;

  // Check home button state and determine if a debounce action occurs and toggles the relay
  if (gamepad->miscSystem()) {

    relayDebounce = true;

  } else {

    if (relayDebounce) {

      relayDebounce = false;
      relayState = !relayState;
      relayState ? cmdEnable() : cmdDisable();

      Serial.println("Relay toggled");
    }
  }

  // Check relay state and process the remaining gamepad controls if it is high
  if (relayState) {

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
    
    } else if (gamepad->y()) {

      Serial.println("[ESP32]: (X) pressed");
    
    } else if (gamepad->x()) {

      Serial.println("[ESP32]: (Y) pressed");
    
    } else if (gamepad->dpad() & DPAD_UP) {

      Serial.println("[ESP32]: DP-UP pressed");
    
    } else if (gamepad->dpad() & DPAD_RIGHT) {

      Serial.println("[ESP32]: DP-RIGHT pressed");
    
    } else if (gamepad->dpad() & DPAD_DOWN) {

      Serial.println("[ESP32]: DP-DOWN pressed");

    } else if (gamepad->dpad() & DPAD_LEFT) {
    
      Serial.println("[ESP32]: DP-LEFT pressed");
      
    } else if (gamepad->l1()) {
    
      Serial.println("[ESP32]: L-Shoulder pressed");
      
    } else if (gamepad->l2()) {  
    
      Serial.println("[ESP32]: L-Trigger pressed");
      
    } else if (gamepad->r1()) {
    
      Serial.println("[ESP32]: R-Shoulder pressed");
      
    } else if (gamepad->r2()) {
    
      Serial.println("[ESP32]: R-Trigger pressed");
      
    } else if (gamepad->thumbL()) {

      Serial.println("[ESP32]: L-Thumb pressed");

    } else if (gamepad->thumbR()) {

      Serial.println("[ESP32]: R-Thumb pressed");

    } else if (gamepad->miscBack()) {

      Serial.println("[ESP32]: (-) pressed");

    } else if (gamepad->miscHome()) {

      Serial.println("[ESP32]: (+) pressed");

    } else if (ly < -500 || ry < -500) {
    
      Serial.println("[ESP32]: Walk forward");
      
    } else if (ly > 500 || ry > 500) {  
    
      Serial.println("[ESP32]: Walk backward");
      
    } else if (lx > 500) {
    
      Serial.println("[ESP32]: Spin right");
      
    } else if (lx < -500) {

      Serial.println("[ESP32]: Spin left");
  
    } else if (rx < -500) {

      Serial.println("[ESP32]: Strafe left");
      
    } else if (rx > 500) {

      Serial.println("[ESP32]: Strafe right");
      
    } else {
      
      Serial.println("[ESP32]: Sent idle command");
      actionIdle();

    }
  }
}

void actionCrouch(ControllerPtr gamepad) {

  sendCommand(RETURN_NONE, CMD_PULSE);
  setServoCluster(crouchAngles, crouchMask);

  while(!gamepad->a()) {BP32.update(); delay(50);}

}

void actionIdle() {
  
  sendCommand(RETURN_VA, CMD_PULSE);
  setServoCluster(idleAngles, idleMask);
}

void cmdEnable() {

  sendCommand(RETURN_NONE, CMD_ENABLE);
  setServoCluster(idleAngles, idleMask); //0b11110000000011111

}

void cmdDisable() {

  sendCommand(RETURN_NONE, CMD_DISABLE);
}

void sendCommand(int returnType, int cmd) {
  servoSerial.write(returnType << 4 | cmd);
}

void setServoCluster(float angles[], int pinMask) {
  
  int pIndex = 0;
  for (int i = 0; i < 18; i++) {

    uint8_t mask = 1 << i;
    if (pinMask & mask) {

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
