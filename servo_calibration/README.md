# Servo Calibration
I created a simple tool to map the servo PWM values (in microseconds) and map them to angular positions.  This was created because a problem I noticed with Endeavor I was that the servos did not behave the same way.  

When you control a servo, you're often calling servo.write(angle) or an equivalent in other platforms.  What's really going on is that the angle is converted to a pulse width value, which is what's actually sent as a signal to the servo.  However, this convertion is based on an assumed mapping.  Due to things like product quality and manufacturing variations, what may be considered as "45 degrees" may vary by as large as 200 usec pulse difference.

As such, I've designed a jig that consists of a servo dial and an angle measurement indicator.  We use this with the servo_calibration.ino code that I wrote in order to map the intended angles to their actual pulse values.

## Requirements
* Arduino (doesn't matter what model, just needs to work!)
* Serial terminal, PuTTY preferred for live key input
* Servo
* Servo Horn ([Use this] or somehting of equivalent dimensions)

[Use this]:https://www.amazon.com/dp/B07D56FVK5?ref_=ppx_hzsearch_conn_dt_b_fed_asin_title_2

## Wiring
Nothing too crazy, just hook up the servo to the Arduino's D7 and GND, then wire the servo to an external PSU set to servo voltage (typically 7.4v).

<img src="./images/servo_calibration_wiring.png" height="500" />

## Instructions
1. Mount 3D printed jig parts to servo, the servo jig goes down from the top (be sure the servo axis is centered), and the servo dial takes in M3 bolts (flathead preferred)
2. Turn on PSU, set voltage to servo range, and connect to servo
3. Plug in USB from your computer to Arduino and open up the serial terminal
4. Follow the instructions in the terminal...
    * Physically adjust the servo horn so that the dial is as close to the neutral position as possible
    * Using the terminal, move the servo so that the dial is at neutral position (~1500 usec); Press SPACE to confirm
    * Using the terminal, move the servo so that the dial is at max position (+90/~2500 usec); Press SPACE to confirm
    * Using the terminal, move the servo so that the dial is at min position (-90/~500  usec); Press SPACE to confirm
    * Run a sanity check by using the terminal to control the servo angle (calibrates to the mapped pulse value automatically).  Check with the jig markings to make sure mostly everything lines up (NOTE: RC servos are inherently unfit for high precision robotics, so expect some error.  At the very least your neutral and min/max values should be accurate)
    * Record the neutral and min/max calibration values (you might want to label the servos as well to keep track).  These will be used to write the robot's firmware.

<img src="./images/servo_jig_annotate.png" height="500" />

## Other Notes
* If you're calibrating a 270-degree servo, the instructions are the same, but instead of +/- 90 deg as the min/max, calibrate to the +/- 135 deg marks (one jig mark further).  When running the sanity check, 30 degrees in the terminal correspond to 45 degrees on the servo; the code was made for 180 degrees, which make up the majority of servos, thus didn't justify accommodating for both.