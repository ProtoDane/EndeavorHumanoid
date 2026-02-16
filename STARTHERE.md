## Disclaimer
**This is a work-in-progress project.  Expect issues to be present and changes to be made to this repository.**

**Furthermore, this project involves work with lithium batteries and minor soldering.  Please do not attempt building this project unless you are confident with the risks involved.**
## Requirements

**[Bluepad32]**: Library for ESP32 microcontrollers to connect to Bluetooth HID devices.  See link for installation instructions.

**[Pico SDK]** (Optional): Raspberry Pi Pico (RP2040) C++ libraries and packages to build binary files.  Refer to [this video] for installation to VSCode.

**[Pimoroni SDK]** (Optional): Servo2040-specific libraries and packages

## Assembly Instructions
Refer to the following docs in order:

1. Bill of Materials (BOM)
   * Important: Check the dimensions of the servo before moving to the later steps, especially if purchasing from a vendor different from the BOM. 

2. [Servo Calibration Instructions]
3. Build Instructions
   * You can also refer to the STEP file to see how the assembly comes together.
4. Electronics Layout + Example Wiring
   * Known issues: ESP32 resets upon relay trigger.  Currently looking into it; consider setting up the relay on its own 5V buck converter.
5. Programming Instructions

[Servo Calibration Instructions]:./docs/ServoCalibrationDoc.pdf
[Bluepad32]:https://bluepad32.readthedocs.io/en/latest/
[Pico SDK]:https://github.com/raspberrypi/pico-sdk
[Pimoroni SDK]:https://github.com/pimoroni/pimoroni-pico
[this video]:https://www.youtube.com/watch?v=B5rQSoOmR5w