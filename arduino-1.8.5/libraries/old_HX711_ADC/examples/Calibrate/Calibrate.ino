//-------------------------------------------------------------------------------------
// HX711_ADC.h
// Arduino master library for HX711 24-Bit Analog-to-Digital Converter for Weigh Scales
// Olav Kallhovd sept2017
// Tested with      : HX711 asian module on channel A and YZC-133 3kg load cell
// Tested with MCU  : Arduino Nano
//-------------------------------------------------------------------------------------
/* This is an example sketch on how to find correct calibration factor for your HX711:
   - Power up the scale and open Arduino serial terminal
   - After stabelizing and tare is complete, put a known weight on the load cell
   - Observe values on serial terminal
   - Adjust the calibration factor until output value is same as your known weight:
      - Sending 'l' from the serial terminal decrease factor by 1.0
      - Sending 'L' from the serial terminal decrease factor by 10.0
      - Sending 'h' from the serial terminal increase factor by 1.0
      - Sending 'H' from the serial terminal increase factor by 10.0
      - Sending 't' from the serial terminal call tare function
   - Observe and note the value of the new calibration factor
   - Use this new calibration factor in your sketch
*/

#include <HX711_ADC.h>

//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell(4, 5);

long t;

void setup() {
  Serial.begin(9600);
  Serial.println("Wait...");
  LoadCell.begin();
  long stabilisingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilising time
  LoadCell.start(stabilisingtime);
  LoadCell.setCalFactor(696.0); // user set calibration factor (float)
  Serial.println("Startup + tare is complete");
}

void loop() {
  //update() should be called at least as often as HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS
  //longer delay in scetch will reduce effective sample rate (be carefull with delay() in loop)
  LoadCell.update();

  //get smoothed value from data set + current calibration factor
  if (millis() > t + 250) {
    float i = LoadCell.getData();
    float v = LoadCell.getCalFactor();
    Serial.print("Load_cell output val: ");
    Serial.print(i);
    Serial.print("      Load_cell calFactor: ");
    Serial.println(v);
    t = millis();
  }

  //receive from serial terminal
  if (Serial.available() > 0) {
    float i;
    char inByte = Serial.read();
    if (inByte == 'l') i = -1.0;
    else if (inByte == 'L') i = -10.0;
    else if (inByte == 'h') i = 1.0;
    else if (inByte == 'H') i = 10.0;
    else if (inByte == 't') LoadCell.tareNoDelay();
    if (i != 't') {
      float v = LoadCell.getCalFactor() + i;
      LoadCell.setCalFactor(v);
    }
  }

  //check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }

}