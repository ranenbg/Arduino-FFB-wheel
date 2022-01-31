# Arduino-FFB-wheel
Stand alone USB device recognized as a joystick with force feedback functionality, based on BR Wheel by Fernando Igor from 2017.

Some firmware features:
- suported boards: Leonardo, Micro, Pro Micro
- 4 analog axis + 1 for optical encoder
- 16 buttons via button box firmware uploaded to Arduino Nano/Uno
- fully supported 16bit FFB effects (custom force not included)
- envelope and conditional block effects, start delay and durration
- FFB calculation and axis/buttons update rate is 500Hz
- many options available (external 12bit ADC/DAC, pedal autocalibration, z-index, hatswitch)
- serial interface for configuration of all wheel parameters
- fully adjustable FFB output in the form of PWM or DAC signals
- load cell support for HX711 chip
- wheel control user interface for an easy configuration and monitoring of all inputs

Firmware v17x pinouts
![plot](./brWheel_my/Firmware-v17x%20pinout.png)
Button box firmware pinouts
![plot](./brWheel_my/Firmware-v141%20button%20box%20pinout.png)
