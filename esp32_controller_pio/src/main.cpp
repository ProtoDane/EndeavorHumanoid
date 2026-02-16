
// Include System libraries
#include <Arduino.h>
#include <uni.h>
#include <Bluepad32.h>
#include <PID_v1.h>
#include <Adafruit_BNO08x.h>

// Include User-Defined Libraries
#include "imu.h"
#include "servo2040.h"
#include "systemParams.h"
#include "config.h"
#include "actions.h"

HardwareSerial IMUSerial(1);
HardwareSerial servoSerial(2);
Adafruit_BNO08x bno08x(-1);

// Control variables
bool operable = true;
bool enableBalancing = false;
volatile bool relayState = false;
volatile bool tipSafetyEnabled = false;
bool isIdle = false;

// PID
double Kp = 1.0; // go lower
double Ki = 0.0;
double Kd = 0.02;
double pidSet, pidIn, pidOut;
PID pidPitch(&pidIn, &pidOut, &pidSet, Kp, Ki, Kd, DIRECT);

// Dual core instances
TaskHandle_t imuTaskHandle;
TaskHandle_t serialTaskHandle;
QueueHandle_t imuQueue;
QueueHandle_t serialQueue;


servoHandler servo;
imuHandler imu;
actionHandler actions;

ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

// ==========================================================================
// Helper Functions
// ==========================================================================
void queueSerial(int input) {
    xQueueSend(serialQueue, &input, 0);
}

// ==========================================================================
// Bluepad32 Handler Functions
// ==========================================================================

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

// Gamepad input handler
void gamepadProcessor(ControllerPtr gamepad) {
    
    // Check home button state and determine if a debounce action occurs and toggles the relay
    if (gamepad->miscSystem() & operable) {

        while(gamepad->miscSystem()) {BP32.update(); delay(50);}

        relayState = !relayState;

        if (!SERIAL_DEBUG_MODE) {
            digitalWrite(RELAY_PIN, !digitalRead(RELAY_PIN));
            digitalWrite(HEAD_PIN, !digitalRead(HEAD_PIN));
        }

        relayState ? servo.cmdEnable() : servo.cmdDisable();
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
        
            // Serial.println("[ESP32]: (B) pressed");
            tipSafetyEnabled = true;
            isIdle = false;
            actions.actionCrouch(gamepad);

        } else if (gamepad->b()) {

            // Serial.println("[ESP32]: (A) pressed");
            tipSafetyEnabled = true;
            isIdle = false;
            actions.actionSwipe(gamepad, false);
        
        } else if (gamepad->y()) {

            // Serial.println("[ESP32]: (X) pressed");
            if(!ULT_LOCK) {
                tipSafetyEnabled = false;
                actions.actionTumble(gamepad);
                tipSafetyEnabled = true;
                isIdle = false;
            }
        
        } else if (gamepad->x()) {

            // Serial.println("[ESP32]: (Y) pressed");
            tipSafetyEnabled = true;
            isIdle = false;
            actions.actionSwipe(gamepad, true);
        
        } else if (gamepad->dpad() & DPAD_UP) {

            // Serial.println("[ESP32]: DP-UP pressed");
            tipSafetyEnabled = true;
            isIdle = false;
            actions.emoteHero(gamepad);
        
        } else if (gamepad->dpad() & DPAD_DOWN) {

            // Serial.println("[ESP32]: DP-DOWN pressed");
            //actionEmote2(gamepad);
            tipSafetyEnabled = true;
            isIdle = false;
            actions.emoteGangnam(gamepad);

        } else if (gamepad->dpad() & DPAD_LEFT) {
        
            // Serial.println("[ESP32]: DP-LEFT pressed");
            tipSafetyEnabled = true;
            isIdle = false;
            actions.emoteShikanoko(gamepad);

        } else if (gamepad->dpad() & DPAD_RIGHT) {

            // Serial.println("[ESP32]: DP-RIGHT pressed");
            tipSafetyEnabled = true;
            isIdle = false;
            actions.emoteTaunt(gamepad);

        } else if (gamepad->l1()) {
        
            // Serial.println("[ESP32]: L-Shoulder pressed");
            tipSafetyEnabled = true;
            isIdle = false;
            actions.actionPunch(gamepad, true);
        
        } else if (gamepad->l2()) {  
        
            // Serial.println("[ESP32]: L-Trigger pressed");
            tipSafetyEnabled = true;
            isIdle = false;
            actions.actionUpperCut(gamepad, true);
        
        } else if (gamepad->r1()) {
        
            // Serial.println("[ESP32]: R-Shoulder pressed");
            tipSafetyEnabled = true;
            isIdle = false;
            actions.actionPunch(gamepad, false);
        
        } else if (gamepad->r2()) {
        
            // Serial.println("[ESP32]: R-Trigger pressed");
            tipSafetyEnabled = true;
            isIdle = false;
            actions.actionUpperCut(gamepad, false);
        
        // } else if (gamepad->thumbL()) {
        // } else if (gamepad->thumbR()) {
        // } else if (gamepad->miscBack()) {   // (-) Button Pressed

        } else if (gamepad->miscHome()) {   // (+) Button Pressed

            // actionGetupFront(gamepad);
            actions.actionGetup(gamepad);
            isIdle = false;
            tipSafetyEnabled = true;

        } else if (ly < -AXIS_THRESHOLD || ry < -AXIS_THRESHOLD) {
        
            Serial.println("[ESP32]: Walk forward");
            tipSafetyEnabled = true;
            isIdle = false;
            actions.moveWalkFwd(gamepad);
        
        } else if (ly > AXIS_THRESHOLD || ry > AXIS_THRESHOLD) {  
        
            Serial.println("[ESP32]: Walk backward");
            tipSafetyEnabled = true;
            isIdle = false;
            actions.moveWalkBwd(gamepad);
        
        } else if (lx > AXIS_THRESHOLD) {
        
            Serial.println("[ESP32]: Spin right");
            tipSafetyEnabled = true;
            isIdle = false;
            actions.moveSpin(gamepad, false);
        
        } else if (lx < -AXIS_THRESHOLD) {

            Serial.println("[ESP32]: Spin left");
            tipSafetyEnabled = true;
            isIdle = false;
            actions.moveSpin(gamepad, true);
    
        } else if (rx < -AXIS_THRESHOLD) {

            Serial.println("[ESP32]: Strafe left");
            tipSafetyEnabled = true;
            isIdle = false;
            actions.moveStrafe(gamepad, true);
        
        } else if (rx > AXIS_THRESHOLD) {

            Serial.println("[ESP32]: Strafe right");
            tipSafetyEnabled = true;
            isIdle = false;
            actions.moveStrafe(gamepad, false);
        
        } else {
            // Serial.println("IDLE");
            // if (!isIdle) {
            //     actions.actionIdle();
            //     isIdle = !isIdle;
            // }
            actions.actionIdle();
        }


    }
}

// ==========================================================================
// Core 1 FreeRTOS tasks
// ==========================================================================

void serialOutTask(void *params) {

    double filteredPitch = 0.0;
    int input;
    enum {READ_CMD, READ_IMU} state = READ_CMD;
    uint8_t buffer[24];
    bool loopContents = false;

    while(1) {
        
        loopContents = false;

        if (xQueueReceive(serialQueue, &input, 0) == pdTRUE) {
            Serial.println(input);
            servoSerial.write(input);
            loopContents = true;
        }

        if (state == READ_CMD) {
            // Read one byte for command
            if (servoSerial.available() > 0) {
                if (servoSerial.read() == 0b10010000) {
                    state = READ_IMU;
                    loopContents = true;
                }
            }
        } else if (state == READ_IMU) {
            // Read 24 bytes for IMU buffer
            if (servoSerial.available() >= 24) {
                servoSerial.readBytes(buffer, 24);
                state = READ_CMD;

                queueBin msg;
                memcpy(&msg.eulerX, &buffer[0], 8); // roll
                memcpy(&msg.eulerY, &buffer[8], 8); // pitch
                memcpy(&msg.eulerZ, &buffer[16], 8); // yaw

                filteredPitch = 0.5 * msg.eulerY + (1 - 0.5) * filteredPitch;
                pidIn = filteredPitch;
                pidSet = 0;
                pidPitch.Compute();
                msg.pidOut = pidOut;
                msg.dX = 125.0 * tan(radians(pidOut));
                
                xQueueOverwrite(imuQueue, (void *) &msg);

                Serial.println(msg.dX);

                if (tipSafetyEnabled && abs(filteredPitch) > IMU_TIP_THRESHOLD && FALL_PROTECTION_ENABLED) {
                    relayState = false;
                    tipSafetyEnabled = false;

                    if (!SERIAL_DEBUG_MODE) {
                        digitalWrite(RELAY_PIN, LOW);
                        digitalWrite(HEAD_PIN, LOW);
                    }
                }
                
                loopContents = true;
            }
        }

        if (!loopContents) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

void serialInTask(void *params) {

    double filteredPitch = 0.0;
    int input;

    while(1) {

        vTaskDelay(pdMS_TO_TICKS(1));

        if (servoSerial.available()) {
            int input = servoSerial.read();

            if (input == 0b10010000) {

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

                xQueueOverwrite(imuQueue, (void *) & msg);

                if (tipSafetyEnabled && abs(filteredPitch) > IMU_TIP_THRESHOLD && FALL_PROTECTION_ENABLED) {
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
}

void imuTask(void *params) {

    pidPitch.SetMode(AUTOMATIC);
    pidPitch.SetOutputLimits(-20, 20);
    pidPitch.SetSampleTime(10);

    // vTaskDelay(pdMS_TO_TICKS(3000));

    euler_t ypr;
    double filteredPitch = 0.0;

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
        imu.process_bno08x(&ypr);
        
        queueBin msg;
        filteredPitch = 0.5 * ypr.pitch + (1 - 0.5) * filteredPitch;
        pidIn = -1 * filteredPitch;
        pidSet = 0;
        pidPitch.Compute();
        msg.eulerY = -1 * filteredPitch;
        msg.pidOut = pidOut;
        msg.dX = 125.0 * tan(radians(pidOut));

        xQueueOverwrite(imuQueue, (void*) &msg);

        if (tipSafetyEnabled && abs(filteredPitch) > IMU_TIP_THRESHOLD && FALL_PROTECTION_ENABLED) {
            relayState = false;
            tipSafetyEnabled = false;

            if (!SERIAL_DEBUG_MODE) {
                digitalWrite(RELAY_PIN, LOW);
                digitalWrite(HEAD_PIN, LOW);
            }
        }
    }
}

// ==========================================================================
// Core 0 FreeRTOS tasks
// ==========================================================================

// ==========================================================================
// Arduino Super Loop
// ==========================================================================

void setup() {
    // put your setup code here, to run once:
    delay(3000);

    pinMode(RELAY_PIN, OUTPUT);
    pinMode(HEAD_PIN, OUTPUT);

    Serial.begin(115200);
    servoSerial.begin(115200, SERIAL_8N1, 16, 17);

    Serial.println("1");

    // xQueue initialization
    imuQueue = xQueueCreate(1, sizeof(struct queueBin));
    serialQueue = xQueueCreate(512, sizeof(int));

    if (!servo.begin(serialQueue)) {
        while(1) {delay(1000); Serial.println("Servo2040 initialization failed");}
    }

    Serial.println("2");
    // Bluepad32 Setup
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    bd_addr_t controller_addr;
    sscanf_bd_addr(CTRL_MAC, controller_addr);
    uni_bt_allowlist_add_addr(controller_addr);
    uni_bt_allowlist_set_enabled(true);
    String fv = BP32.firmwareVersion();
    Serial.print("BP32 Firmware Version: ");
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

    Serial.println("3");

    if (IMU_CONFIG == BNO08x) {
        Serial.println("Using BNO08x IMU");
        if (!imu.begin()) {
            while(1) {delay(1000); Serial.println("IMU initialization failed");}
        }

        xTaskCreatePinnedToCore(imuTask, "imuTask", 8192, NULL, 2, &imuTaskHandle, 0);
    }

    pidPitch.SetMode(AUTOMATIC);
    pidPitch.SetOutputLimits(-20, 20);
    pidPitch.SetSampleTime(10);

    // FreeRTOS tasks
    // xTaskCreatePinnedToCore(serialInTask, "serialInTask", 10000, NULL, 1, &serialTaskHandle, 0);
    xTaskCreatePinnedToCore(serialOutTask, "serialTask", 10000, NULL, 3, &serialTaskHandle, 0);

    actions.begin(servo, imuQueue);

    queueSerial(0b01101001);

    Serial.println("Ready!");
    digitalWrite(HEAD_PIN, HIGH);
    delay(500);
    digitalWrite(HEAD_PIN, LOW);
}

void loop() {
    // put your main code here, to run repeatedly:
    BP32.update();

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        ControllerPtr gamepad = myControllers[i];
        if (gamepad != nullptr) {
            gamepadProcessor(gamepad);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}