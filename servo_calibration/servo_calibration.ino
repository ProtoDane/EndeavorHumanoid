// Servo_calibration by ProtoDane
// Version 1 | 2025-02-27

// Hook up an Arduino to a USB port and external power for the servo (6-7.4V) and use a serial monitor (PuTTY recommended) to 
// measure the pulse width values of the servo to the angle measurements marked out on the jig.
//
// Requires jig, see GitHub repo
//
// Note: Neutral is the vertical center position of the servo, referred often as zero.  For 180-degree servos, the min/max values
//       you want to calibrate are at +/-90 degrees, or two marks from the neutral.  For 270-degree servos, those values are at
//       three marks from neutral; this code is made for 180 degrees, so 45 degrees on the physical setup corresponds to 30 degrees
//       in the code.  Keep that in mind when doing the sanity check!

#include <Servo.h>

// Constants, feel free to edit based on your servo specs and Arduino board
#define SERIAL_BAUD 9600
#define SERVO_PIN 7
#define SERVO_MIN_LIMIT 500
#define SERVO_MAX_LIMIT 2500

Servo testServo;
int servoPulse;

// Calibration pulse width value storage variables
int calibFirstVal;
int calibLastVal;
int calibNeutralVal;

// State machine values to control the process flow of the calibration sequence
enum State {calibInit, calibMid, calibFirst, calibLast, calibEnd};
enum State currentState = calibInit;

void setup() {

  Serial.begin(SERIAL_BAUD);

  while (!Serial) {;} // Hold until serial port is connected
  
  testServo.attach(SERVO_PIN, SERVO_MIN_LIMIT, SERVO_MAX_LIMIT);
}

void loop() {

  switch(currentState){
    
    case calibInit:
      
      // Display controls and hold until the user presses any key
      Serial.println("Controls:");
      Serial.println("Q/A: +/- 100 us");
      Serial.println("W/S: +/- 10 us");
      Serial.println("E/D: +/- 1 us");
      Serial.println("R: Reset to neutral");
      Serial.println("SPACE: Confirm calibration value");
      Serial.println("\nPress any key to continue...");
      while(!Serial.available()) {;}
      Serial.read();

      currentState = calibMid;
      break;

    case calibMid:

      // Prompt user to calibrate the neutral position (around 1500 usec for DS3235 servos)
      Serial.println("\nCalibrate the servo to the neutral position");
      calibNeutralVal = calibrateServo();
      Serial.println("Neutral position recorded at " + String(calibNeutralVal));

      currentState = calibFirst;
      break;
    
    case calibFirst:

      // Prompt user to calibrate the max value at +90 (or +135 for 270 degree servos), which is counter-clockwise from neutral
      Serial.println("\nCalibrate the servo to +90 degrees from the neutral position (counter-clockwise)");
      calibFirstVal = calibrateServo();
      Serial.println("+45 degrees recorded at " + String(calibFirstVal));
      
      currentState = calibLast;
      break;
      
    case calibLast:

      // Prompt user to calibrate the min value at -90 (-135 for 270 degree servos), which is clockwise from neutral
      Serial.println("\nCalibrate the servo to -90 degrees from the neutral position (clockwise)");
      calibLastVal = calibrateServo();
      Serial.println("-45 degrees recorded at " + String(calibLastVal));
      
      currentState = calibEnd;      
      break;
    
    case calibEnd:

      // Run subprocess to control the servo by the angle as sanity check (converts to calibrated pulse value)
      Serial.println("\nStarting user control of calibrated servo...");

      actuateCalibratedServo(calibNeutralVal, calibFirstVal, calibLastVal);
      
      Serial.println("\nPrinting results, be sure to save this somewhere...");
      Serial.println("Calibrated neutral value      (us): " + String(calibNeutralVal));
      Serial.println("+90 degrees calibration value (us): " + String(calibFirstVal));
      Serial.println("-90 degrees calibration value (us): " + String(calibLastVal));
      
      Serial.println("Resetting the terminal...\n");
      
      currentState = calibInit;
      break;
  }
}

// Set servo to desired angle given calibration numbers.  If value is above neutral/zero, map between neutral and high.
// If value is below neutral/zero, map between neutral and low.  Otherwise, set servo to neutral.  Returns calibrated pulse.
int setCalibratedServo(int value, int neutral, int high, int low) {

  int pulse;
  
  if(value > 0) {

    pulse = map(value, 0, 90, neutral, high);
    
  } else if(value < 0){

    pulse = map(value, -90, 0, low, neutral);
    
  } else {
    
    pulse = neutral;
  
  }

  testServo.writeMicroseconds(pulse);
  return pulse;
}

// Subroutine function to control the calibrated servo using angle input.
void actuateCalibratedServo(int neutral, int high, int low) {

  char receivedChar = '0';
  int servoPulse = neutral;
  int servoAngle = 0;
  Serial.println("Setting servo to neutral position (" + String(neutral) + "us)...");
  testServo.writeMicroseconds(servoPulse);

  Serial.println("Controls:");
  Serial.println("Q/A: +/- 10 deg");
  Serial.println("W/S: +/-  5 deg");
  Serial.println("E/D: +/-  1 deg");
  Serial.println("R: Reset to neutral");
  Serial.println("SPACE: Quit");

  while(receivedChar != ' ') {

    while(!Serial.available()){;}
    receivedChar = Serial.read();

    switch(receivedChar) {
      case 'q':
        servoAngle += 10;
        break;
        
      case 'a':
        servoAngle += -10;
        break;
        
      case 'w':
        servoAngle += 5;
        break;
        
      case 's':
        servoAngle += -5;
        break;
        
      case 'e':
        servoAngle += 1;
        break;
        
      case 'd':
        servoAngle += -1;
        break;
        
      case 'r':
        Serial.println("Resetting to neutral position (" + String(neutral) + "us)...");
        testServo.writeMicroseconds(neutral);
        servoPulse = neutral;
        break;
        
      case ' ':
        break;
        
      default:
        Serial.println("Unrecognized character!"); 
    }

    if (receivedChar != 'r' && receivedChar != ' '){

      servoAngle = constrain(servoAngle, -90, 90);
      servoPulse = setCalibratedServo(servoAngle, neutral, high, low);
      Serial.print("Current angle: " + String(servoAngle) + "deg ");
      Serial.println("| Corresponding pulse: " + String(servoPulse) + "us");
    }
  }
  
}

// Subroutine function to set the servo pulse to target position (neutral/min/max)
int calibrateServo() {
  
  char receivedChar  = '0';
  int servoPulse = 1500;
  Serial.println("Setting servo to neutral position (1500us)...");
  testServo.writeMicroseconds(servoPulse);

  while(receivedChar != ' ') {

    while(!Serial.available()){;}
    
    receivedChar = Serial.read();
    
    switch(receivedChar){
      case 'q':
        servoPulse += 100;
        break;
        
      case 'a':
        servoPulse += -100;
        break;
        
      case 'w':
        servoPulse += 10;
        break;
        
      case 's':
        servoPulse += -10;
        break;
        
      case 'e':
        servoPulse += 1;
        break;
        
      case 'd':
        servoPulse += -1;
        break;
        
      case 'r':
        Serial.println("Resetting to neutral position (1500us)");
        testServo.writeMicroseconds(1500);
        servoPulse = 1500;
        break;
        
      case ' ':
        break;
        
      default:
        Serial.println("Unrecognized character!");        
    }

    if (receivedChar != 'r' || receivedChar != ' '){

      servoPulse = constrain(servoPulse, SERVO_MIN_LIMIT, SERVO_MAX_LIMIT);
      testServo.writeMicroseconds(servoPulse);
      Serial.println("Current servo pulse (us): " + String(servoPulse));
    }
  }

  return servoPulse;
}
