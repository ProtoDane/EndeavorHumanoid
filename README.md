# Endeavor II - 3D Printed Humanoid Robot
This is a 17-degree of freedom mini humanoid robot that can be 3D printed off a standard machine such as an Ender 3, Prusa, or Bambu.  This is the second iteration, which aims to address the shortcomings and design flaws of its predecessor.  While this robot was specifically designed to compete in ROBO-ONE or similar events (e.g. RoboGames), I want to use this as an experimental platform to explore relevant problems within the humanoid robotics field such as dynamic balancing, advanced motion generation, etc.  This project is intended to be open sourced; CAD files, code, and documentation will follow the project's progress!

<img src="./images/Endeavor2_CAD.png" height="500" />

## Navigation
### servo_calibration
Simple 3D printed jig and Arduino code to calibrate the servos and map the angles to their actual pulse width values.  This tool package is used when we write the firmware for the robot to ensure that the pulse values we send to the servos more or less correspond to the angles we want to set in the joints.
### servo2040_client
Pimoroni Servo2040 code that can control the board using a UART-based command protocol.  Allows for individual servo control, built-in current and voltage feedback to host device, and LED control.
### esp32_basic_controller
ESP32 Arduino-based code that uses the Bluepad32 library to connect to a Bluetooth game controller.  Depending on which buttons or joysticks are being used, the ESP32 sends different UART messages to actuate the servos.
### jupyter_notebook > ik_generator.ipynb
Python-based leg inverse kinematics generator.  This is used to produce the leg animations that are stored in the ESP32. (It's a bit of a mess, and I'm working on cleaning it up!)
## Requirements
[Bluepad32]: Library for ESP32 microcontrollers to connect to Bluetooth HID devices

[Pico SDK]: Raspberry Pi Pico (RP2040) SDK  to provide the libraries and build tools necessary to write code in C/C++

[Pimoroni SDK]: Pimoroni SDK to provide specific libraries and tools for Pimoroni boards such as the Servo2040

*Note: the uf2 file for the Servo2040 is provided, so the SDK's are not needed unless you want to write your own firmware for the Servo2040.*

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