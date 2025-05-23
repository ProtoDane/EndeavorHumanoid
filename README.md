# Endeavor II - 3D Printed Humanoid Robot
This is a 17-degree of freedom mini humanoid robot that can be 3D printed off a standard machine such as an Ender 3, Prusa, or Bambu.  This is the second iteration, which aims to address the shortcomings and design flaws of its predecessor.  While this robot was specifically designed to compete in ROBO-ONE or similar events (e.g. RoboGames), I want to use this as an experimental platform to explore relevant problems within the humanoid robotics field such as dynamic balancing, advanced motion generation, etc.  This project is intended to be open sourced; CAD files, code, and documentation will follow the project's progress!

<img src="./images/Endeavor2_CAD.png" height="500" />

## Navigation
### servo_calibration
Simple 3D printed jig and Arduino code to calibrate the servos and map the angles to their actual pulse width values.  This tool package is used when we write the firmware for the robot to ensure that the pulse values we send to the servos more or less correspond to the angles we want to set in the joints.
### servo2040_client
Pimoroni Servo2040 code that can control the board using a UART-based command protocol.  Allows for individual servo control, built-in current and voltage feedback to host device, and LED control.
### servo2040_client_imu
Variant of the original client firmware to poll an Adafruit BNO-055 9DOF IMU sensor.  IMU data is sent to the ESP32 on a 20Hz clock cycle using the second core of the RP2040
### esp32_controller_basic
ESP32 Arduino-based code that uses the Bluepad32 library to connect to a Bluetooth game controller.  Depending on which buttons or joysticks are being used, the ESP32 sends different UART messages to actuate the servos.
### esp32_controller_imu
Variant of original ESP32 controller sketch that interacts with the Servo2040 with the IMU client.  IMU data is received and used to perform simple tasks such as deactivating the servos when a tip is detected, and getting up from a fall under a consolidated gamepad button (uses pitch angle to determine whether robot is facing up or down).
### esp32_controller_dynamic
Coming soon!
### jupyter_notebook > ik_generator.ipynb
Python-based leg inverse kinematics generator.  This is used to produce the leg animations that are stored in the ESP32. (It's a bit of a mess, and I'm working on cleaning it up!)
## Requirements
### Software
[Bluepad32]: Library for ESP32 microcontrollers to connect to Bluetooth HID devices

[Pico SDK]: Raspberry Pi Pico (RP2040) C++ libraries and packages to build binary files

[Pimoroni SDK]: Servo2040-specific libraries and packages

[Bluepad32]:https://bluepad32.readthedocs.io/en/latest/
[Pico SDK]:https://github.com/raspberrypi/pico-sdk
[Pimoroni SDK]:https://github.com/pimoroni/pimoroni-pico
## Documentation
Coming soon!
## Contributing
I have a [Discord server] where you can leave feedback, get support for your project, and hang out with robotics fanatics!

[Discord server]:https://discord.gg/Gm2sCxpUSx

## Supporting
TBD