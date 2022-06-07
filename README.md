# Arduino-FFB-wheel
Stand alone USB device recognized as a joystick with force feedback functionality, based on BRWheel by Fernando Igor from 2017.

Firmware features:
- supported Arduino boards: Leonardo, Micro and ProMicro (5V, 16MHz)
- 4 analog axis + 1 for optical encoder, 2 FFB axis (only 1 has pwm or dac output)
- up to 16 buttons via by 4x4 matrix or by **[button box firmware](https://github.com/ranenbg/Arduino-FFB-wheel/tree/master/tx_rw_ferrari_458_wheel_emu_16buttons)** uploaded to Arduino Nano/Uno
- fully supported 16bit FFB effects (custom force effect not implemented)
- envelope and conditional block effects, start delay, durration, deadband
- FFB calculation and axis/button update rate is 500Hz (2ms period)
- many options available (external 12bit ADC/DAC, pedal autocalibration, z-index, hatswitch, button matrix)
- RS232 serial interface for configuration of all wheel parameters
- fully adjustable FFB output in the form of 2 channel digital PWM or analog DAC signals
- load cell support for HX711 chip (for brake pedal axis only)
- all wheel parameters are stored in EEPROM (and automatically loaded at each powerup)
- wheel control **[Arduino FFB gui](https://github.com/ranenbg/Arduino-FFB-gui)** for an easy configuration and monitoring of all inputs/outputs 

Detailed documentation and more information about the firmware can be found in txt files inside brWheel_my folder.

# Firmware pinouts and wiring diagrams
![plot](./brWheel_my/Firmware-v18x%20pinout.png)
## Button box firmware pinouts - for Arduino Nano/Uno
![plot](./brWheel_my/Firmware-vXX1%20button%20box%20pinout.png)
## Button matrix pinouts
![plot](./brWheel_my/button_matrix_wiring_diagram.png)
## External i2C device pinouts
![plot](./brWheel_my/ads1015_wiring_diagram.png)
![plot](./brWheel_my/mcp4725_wiring_diagram.png)
## Motor driver wiring
![plot](./brWheel_my/bts7960_wiring_diagram.png)

## Firmware option description
Due to 32k flash memory limitation in Arduino Leonardo (ATmega32U4), each HEX file is compiled with a certain firmware option. A one letter abreviation for each option is placed in the firmware version string and one needs to consider carefully which one to chose. In the release, I've compiled for you a few most often desired firmware option combinations.

Firmware version string consits of 3 digits and some letters (example: fw-v180ahz). The first two digits (XX) are reserved for major firmware version, while the 3rd digit (0,1,2,3) stands for:
- fw-vXX0 basic version (1 encoder, 4 pedals, 8 buttons)
- fw-vXX1 adds support for shift register 
- fw-vXX2 adds support for shift register+HX711
- fw-vXX3 adds support for shift register+HX711+MC7425 analog DAC

 Here is the complete list of all available options that may be added to any of the above firmware*:
- "a" pedal axis autocalibration enabled
- "z" Z-index encoder support
- "h" Hat Switch support
- "s" external ADC support for pedals with ADS1015
- "i" averaging of analog inputs
- "t" 4x4 button matrix support
- "m" replacement pinouts for Arduino ProMicro

note* some combinations are not possible at the same time like "z" and "s", beacause they both use the same Arduino pin

## Firmware download

+ ***[Latest Release](https://github.com/ranenbg/Arduino-FFB-wheel/releases/latest)***
+ ***[Past Versions](https://github.com/ranenbg/Arduino-FFB-wheel/releases)***

## Firmware upload procedure
You can use **[XLoader](https://github.com/ranenbg/Arduino-FFB-wheel/tree/master/XLoader)**:
- set 57600baud, ATMega32U4 microcontroler and select desired HEX
- press reset button on Arduino (or shortly connect RST pin to GND)
- select newly appeared COM port (Arduino in bootloader mode*) and press upload, you will only have a few seconds

*It is possible that some cheap chinese clones of Arduino Leonardo, Micro or ProMicro do not have a bootloader programmed. In that case you need to upload the original Arduino Leonardo bootloader first. You can find more details about it here: https://docs.arduino.cc/built-in-examples/arduino-isp/ArduinoISP

## Credits

Original BRWheel firmware by: Tero Loimuneva, Saku Kekkonen, Etienne Saint-Paul and Fernando Igor.
https://github.com/fernandoigor/BRWheel/tree/alphatest
