#include "servo2040.h"

bool servoHandler::begin(QueueHandle_t serialQueue) {
    _serialQueue = serialQueue;
    return true;
}

void servoHandler::actuate(double pidOutput) {
}

void servoHandler::cmdEnable() {
    Serial.println("Enabling Servo2040...");
    sendCommand(RETURN_NONE, CMD_ENABLE);
    setServoCluster(idleAngles, ALL_SERVOS); //0b11110000000011111

    delay(500);
}

void servoHandler::cmdDisable() {
    Serial.println("Disabling Servo2040...");
    sendCommand(RETURN_NONE, CMD_DISABLE);
}

void servoHandler::sendCommand(int preamble, int cmd) {
    queueWrite(preamble << 4 | cmd);
}

void servoHandler::setServo(int pulse) {
    int first = pulse / 100;
    int second = pulse % 100;
    queueWrite(first);
    queueWrite(second);
}

// Sets the servo cluster to the desired list of angles.  pinMask is used to identify which
// servos are to be actuated
void servoHandler::setServoCluster(const float* angles, int pinMask) {

    int pIndex = 0; 

    for (int i = 0; i < 18; i++) {

        uint32_t mask = 1 << i;

        if (pinMask & mask) {

            servoStruct servo = servoCluster[i];
            int calibratedPulse = mapPulse(angles[pIndex], servo.min, servo.mid, servo.max, servo.angle180);
            setServo(calibratedPulse);
            pIndex++;
        } else {

            setServo(1);
        }
    }
}

void servoHandler::setServoCluster(legAngles *l, armAngles *a, float torsoAngle) {
    float angles[17];
    angles[0] = torsoAngle;
    angles[1] = a->ra1;
    angles[2] = a->ra2;
    angles[3] = a->ra3;
    angles[4] = a->ra4;
    angles[5] = l->rth1;
    angles[6] = l->rth2;
    angles[7] = l->rth3;
    angles[8] = l->rth1;
    angles[9] = l->lth1;
    angles[10] = l->lth2;
    angles[11] = l->lth3;
    angles[12] = l->lth1;
    angles[13] = a->la1;
    angles[14] = a->la2;
    angles[15] = a->la3;
    angles[16] = a->la4;

    setServoCluster(angles, ALL_SERVOS);
}

void servoHandler::setServoSequence(int n, const float *sequence, int pinMask, int sequenceLength) {

    int servoCount = 0;
    for (int i = 0; i < 18; i++) {

        uint32_t mask = 1 << i;
        if (pinMask & mask) {servoCount++;}
    }

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

void servoHandler::setServoDelay(const float *angles, int pinMask, int delayMs) {
    int first = delayMs / 100;
    int second = delayMs % 100;
    queueWrite(first);
    queueWrite(second);

    setServoCluster(angles, pinMask);
    vTaskDelay(pdMS_TO_TICKS(delayMs));
}

// Private Functions

int servoHandler::mapPulse(float angle, float min, float mid, float max, bool angle180) {

    float maxAngle = angle180 ? 90.0 : 135.0;
    if (angle > 0.0) {
        return (int) (angle * (max - mid) / maxAngle + mid);
    
    } else if (angle < 0.0) {
        return (int) ((angle + maxAngle) * (mid - min) / maxAngle + min);
    
    } else {
        return mid;
    
    }
}

void servoHandler::queueWrite(int msg) {
    xQueueSend(_serialQueue, &msg, 0);
}