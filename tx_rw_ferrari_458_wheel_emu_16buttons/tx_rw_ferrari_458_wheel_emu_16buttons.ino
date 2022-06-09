/*
  Ver. 1.01 - 2015-06-17
  Ver. 1.02 - 2018-06-17, modified by Milos Rankovic
  Ver. 1.03 - 2022-06-08, modified by Milos Rankovic - button at D8 flicker fix

  This sketch emulates Thrustmaster TX RW 458 Italia wheel
  allowing to connect arduino to TX RW wheelbase (may work for T500/T300 as well)
  and emulate button presses - up to 17 buttons can be connected to arduino, if you
  don't need a serial debugging

  !!! There are 2 flavours of arduino boards - powered by +5 volts or +3.3 volts !!!
  !!! If you have +5V Arduino version - disconnect it from USB before connecting to a wheelbase !!!
  !!! Just power your arduino from TX RW wheelbase Black pin - it supplies +3.3V voltage (should be enough) !!!
  !!! don't connect USB +5V wire to TX RW wheelbase Black +3.3V - you'll get a collision !!!

  Thrustmaster Wheelbase cable pinout (PS/2 connector, also known as mini-Din 6pin female):
  1 - Green    - not used
  2 - Blue     - GND (ground)
  3 - White    - MISO (master in, slave out - to read the data from the wheel)
  4 - Orange   - SS (slave select, or PL - parallel load, set it to 0 when you want to read the data)
  5 - Red      - CLK (clock impulses)
  6 - Black    - +VCC (+3.3 volts! if you have Arduino 5V version - use external +3.3V power or use the wheelbase power!!!)

  Arduino UNO pins -> TX RW wheelbase cable pins

  Arduino GND                 -> TWRX Blue wire (2)
  Arduino pin 12              -> TXRW White wire (3) (data from wheel to base)
  Arduino pin 10 + pin 2 (SS) -> TXRW Orange wire pin (4) (yes, on Arduino it's wired to two pins. 10 - SS, 2 - INT0)
  Arduino pin 13 (SCK)        -> TXRW Red wire (5)
  Arduino +5V                 -> TXRW Black wire (6) (it gives 3.3V, but must be connected into +5V socket on arduino uno side)

  Arduino Leonardo FFB (as wheelbase) -> Arduino Nano or UNO (as button box)

  Leonardo GND                -> Nano GND
  Leonardo 5V                 -> Nano 5V
  Leonardo pin 6 (DT_sh)      -> Nano pin 12
  Leonardo pin 7 (CLK_sh)     -> Nano pin 13
  Leonardo pin 8 (PL_sh)      -> Nano pin 10 and pin 2

  Button mappings (we send 8 bytes to wheelbase, first 16 bits are for buttons, the rest are unused - no more pins on nano)

  Byte 0
  1 – A (button 6)                   -> pin 7 (portD bit7)
  1 – B (button 4)                   -> pin 6 (portD bit6)
  1 – RS (button 12)                 -> pin 5 (portD bit5)
  1 – Menu (button 9)                -> pin 4 (portD bit4)
  1 – Gear Down (button 1 = L_Pad)   -> pin 3 (portD bit3)
  1 – X (button 5)                   -> pin 9 (portB bit1)
  1 – Manettino CCW (button 7, View) -> pin 1 (portD bit1) - if you don't use serial debug (UART TX)
  1 – Manettino CW (button 8, Menu)  -> pin 0 (portD bit0) - if you don't use serial debug (UART RX)

  Byte 1
  1 – Gear Up (button 2 = R_Pad)     -> pin 8 (portB bit0)
  1 – Y (button 3)                   -> pin 11 (portB bit3)
  1 – LS (button 11)                 -> pin A5 (portC bit5)
  1 – View (button 10 – under X)     -> pin A4 (portC bit4)
  1 – D-Pad Down                     -> pin A3 (portC bit3)
  1 - D-Pad Right                    -> pin A2 (portC bit2)
  1 - D-Pad Left                     -> pin A1 (portC bit1)
  1 - D-Pad Up                       -> pin A0 (portC bit0)

  Each push button has 2 pins - you connect one to Arduino, another to the Ground :)
  Pressed button gives you "1" (grounded signal), released = "0" (pulled-up to +VCC)

  Free free to modify and use for your needs.
  This sketch and the documentation above provided "AS IS" under the BSD New license.
  http://opensource.org/licenses/BSD-3-Clause
  (c) Taras Ivaniukovich (blog@rr-m.org) April 2015

  http://rr-m.org/blog/hacking-a-thrustmaster-tx-rw-wheelbase-with-arduino-uno-part-2/
*/

byte wheelState [8];
volatile byte pos;

void setup () {
  DDRB  |= B00001011; // set digital pins 8,9,11 as inputs
  PORTB |= B00001011; // pulled-up to +VCC via internal 100k resistors

  DDRC |= B00111111; // set pins 14-19 (A0-A5) as digital inputs
  PORTC |= B00111111; // pulled-up to +VCC via internal 100k resistors

  DDRD  |= B11111011; // set digital pins 0,1,3,4,5,6,7 as inputs
  PORTD |= B11111011; // pulled-up to +VCC via internal 100k resistors

  //wheelState[0] = B11000000; // TX RW Ferrari 458 Italia Wheel first data byte
  //wheelState[1] = B11111111; // second data byte - buttons
  //wheelState[2] = B11111111; // third data byte - buttons
  wheelState[0] = B00000000; // milos, 1st 8 buttons for my Arduino Leonardo FFB
  wheelState[1] = B00000000; // milos, 2nd 8 buttons for my Arduino Leonardo FFB
  wheelState[2] = B00000000; // milos, 3rd byte not used currently
  wheelState[3] = B00000000; // milos, this and below is not used but T300
  wheelState[4] = B00000000; //        wheelbase reads all 8 bytes
  wheelState[5] = B00000000;
  wheelState[6] = B00000000;
  wheelState[7] = B00000000;

  //Serial.begin(9600);    // Arduino debug console - occupies pins RX (0) and TX (1) on Uno
  pinMode(MISO, OUTPUT); // arduino is a slave device
  SPCR |= _BV(SPE);      // Enables the SPI when 1
  SPCR |= _BV(SPIE);     // Enables the SPI interrupt when 1
  SPCR |= _BV(CPHA);     // milos, added - sample data on rising edge of the clock

  // interrupt for SS rising edge. Arduino Uno Pin10 must be connected to Pin2!!!
  attachInterrupt (0, ss_rising, RISING);
}

// Interrupt0 (external, pin 2) - prepare to start the transfer
void ss_rising () {
  SPDR = wheelState[0]; // load first byte into SPI data register
  pos = 1;
}

// SPI interrupt routine
ISR (SPI_STC_vect) {
  SPDR = wheelState[pos++]; // load the next byte to SPI output register and return.
}

void loop() {
  // scan the button presses and save that to wheelState array. Data transfer to wheelbase is interrupt-driven above.
  //wheelState[0] = (PINB & B00000001) | B11000000;                  // take bit 0 from PORTB - TX RW byte1 // milos, commented
  //wheelState[1] = ((PINB & B00000010) << 1) | (PIND & B11111011); // take bit 1 from PORTB + the rest from PORTD B11111x11 // milos, commented
  //wheelState[2] = ((PINB & B00001000) << 3) | (PINC & B00111111) | B10000000; // take bit 3 from PORTB + bits 0-5 from PORTC // milos, commented

  wheelState[0] = ~(((PINB & B00000010) << 1) | (PIND & B11111011)); // milos, take bit 1 from PINB + the rest from PIND B11111x11
  wheelState[1] = ~(((PINB & B00001000) << 3) | ((PINB & B00000001) << 7) | (PINC & B00111111)); // milos, take bit 3 from PINB and take bit 0 from PINB + bits 0-5 from PINC
}
