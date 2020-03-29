//- -----------------------------------------------------------------------------------------------------------------------
// uA meter with HX711
/*
  PROJECT: uA meter with HX711
  PROGRAMMER: AWI
  DATE: 20170414/last update: 
  LICENSE: Public domain
  SITE: https://www.openhardware.io/view/380/Micro-nano-ampere-meter-double
  original sketch from: https://www.openhardware.io/dl/58ff592144f656d179dca241/code/AWI_uA_meter.ino
  changes see below

Hardware: Arduino Nano (ATmega328p) 5 V

Summary:
  Measures mV accross a shunt resistor ~ uA - channel A
  Measures mV on channel B
  Modes:
    - default: measure uV in full resolution (stable reading only for 0.1 uV)
    - other:
      A: channel A: default, amplification 128 - div 500: 0.1uV stable,  range +/- 20mV, (1ohm +/- 20mA, res 100 nA)
      B: channel B: amplification 32 - div 125: 100nA stable, range +/- 80mV,  (10 ohm +/- 8 mA, res 10 nA)
      AB: both channels:  
    - uA - calibration: depending on the actual shunt:
      0.47 ohm -> 1 uV ~ 2uA, range -40 mA - 40 mA
      1 ohm -> 1 uV = 1uA, range -20 mA - 20 mA
      10 ohm -> 1 uv = 0.1uA
    - mV - calibration, depend on amplification
  Button switch:
    - Short press, reset current channel to offset 0 (keep terminals shorted, no need with uA ;-))
    - Long press, change channel A (uA) / B(uA)/ A & B (uA)

  Hx711 24 bit weight scale sensor
    - Noise and temperature sensitive (x bit effective)
  OLED 128x64 display

Remarks:
    Size is large as result of font library for display

Used libraries:
    https://github.com/olikraus/u8g2/wiki/u8g2install for OLED display
    https://github.com/JChristensen/JC_Button for button
    https://github.com/bogde/HX711 for ADC

Specification HX711:
   The HX711 Dual-Channel 24 Bit Precision A/D weight Pressure Sensor Load Cell Amplifier and 
   ADC Module is a small breakout board for the HX711 IC that allows you to easily read load cells 
   to measure weight.    By connecting the module to your microcontroller you will be able to read the 
   changes in the resistance of the load cell and with some calibration. You?ll be able to get very 
   accurate weight measurements. This can be handy for creating your own industrial scale, process control, 
   or simple presence detection. The HX711 Weighing Sensor uses a two-wire interface (Clock and Data) 
   for communication. Any microcontroller?s GPIO pins should work and numerous libraries have been written 
   making it easy to read data from the HX711. Check the hookup guide below for more information.
   Load cells use a four-wire Wheatstone bridge to connect to the HX711. 
   These are commonly color RED, BLK, WHT, GRN, and YLW.
   Each color corresponds to the conventional color coding of load cells :
      Red (Excitation+ or VCC).
      Black (Excitation- or GND).
      White (Amplifier+, Signal+, or Output+).
      Green (A-, S-, or O-).
      Yellow (Shield).
  The YLW pin acts as an optional input that not hook up to the strain gauge but is utilized to ground 
  and shield against outside EMI (electromagnetic interference).
  Please keep in mind that some load cells might have variations in color-coding.
  Pinouts :
  Analog Side :
      E+: Excitation positive.
      E-: Excitation negative.
      A-: Channel A Negative Input.
      A+: Channel A positive Input.
      B-: Channel B Negative Input.
      B+: Channel B positive Input.
  Digital Side :
      GND: 0 V / Ground Power Connection.
      DT: Data IO Connection.
      SCK: Serial Clock Input.
      VCC: Power Input.
  Specifications and Features :
      Recommended excitation voltage: 5-10 V.
      Two selectable differential input channels.
      On-chip active low noise PGA with a selectable gain of 32, 64 and 128.
      On-chip power supply regulator for load-cell and ADC analog power supply.
      Selectable 10 SPS or 80SPS output data rate.
      On-chip oscillator requiring no external component with optional external crystal.
      On-chip power-on-reset.
      Simple digital control and serial interface: pin-driven controls, no programming needed.
      Simultaneous 50 and 60 Hz supply rejection.
   Examples:
      https://www.rhydolabz.com/sensors-flex-force-c-137_143/load-sensor-amplifier-breakout-hx711-china-p-2058.html
      https://robu.in/product/hx711-weighing-sensor-dual-channel-24-bit-precision-ad-weight-pressure-sensor/
      https://www.instructables.com/id/How-to-Interface-HX711-Balance-Module-With-Load-Ce/

Sources:
  for EEPROM handling: http://www.netzmafia.de/skripten/hardware/Arduino/EEPROM.html

Update:
  v 0.1 (PeMue):
    - change based on these hints https://www.thingiverse.com/thing:3641379
    - added HX11 specifications (juergs)
    - added firmware version
*/
//- -----------------------------------------------------------------------------------------------------------------------
#include <U8g2lib.h>                             // U8glib for OLED display
#include <Wire.h>                                // I2C
//#include <Button.h>                            // https://github.com/JChristensen/Button -> deprecated
#include <JC_Button.h>                           // changes acc. https://www.thingiverse.com/thing:3641379, https://github.com/JChristensen/JC_Button
#include <HX711.h>                               // ADC lib, https://github.com/bogde/HX711

#define FIRMWARE_VER "0.1"
const double calibrationFactorA = 488.5f;        // calibration for channel A: set to 1.0 for known current and divide
const double calibrationFactorB = 122.5f;        // calibration for channel B: set to 1.0 for known current and divide
long offsetChannelA = 0;                         // channel offsets for A and B (drifts) are calibrated at startup and on command 
long offsetChannelB = 0;

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = A4;
const int LOADCELL_SCK_PIN = A5;

const uint8_t HX711_dout = A1;                    // HX711 data out pin
const uint8_t HX711_sck = A0;                     // HX711 serial clock
const uint8_t buttonPin = A2;                     // button to select function and reset offset

/* ???
const unsigned long longPress = 3000UL;           // - long press set reference temperature 
                                                  // - in ms
                                                  // - when alarm, short press resets alarm
// ??? */

Button myBtn(buttonPin, true,  true, 40);         // declare the button (pin, pullup, invert, debounce ms)

enum convertMode_t {channelA, channelB, channelAB};   // measurement modes, 32 port B / 128 port A / A & B

HX711 scale;                                     // instantiate ADC
//HX711 scale(HX711_dout, HX711_sck, 128);       // instantiate ADC, see https://www.thingiverse.com/thing:3641379, does not work

// U8G instantiate, Change this constructor to match the display!!!
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // all boards without reset of the display

const int netReadingsSize = 16;                  // the number of readings to determine the average and calculate variance/accuracy
double lastReading, lastReadingB; 
double netReadings[netReadingsSize];             // store the rolling average of readings
int netReadingPointer = 0; 

convertMode_t convertMode = channelA;            // default channelA

//- -----------------------------------------------------------------------------------------------------------------------
// setup
//- -----------------------------------------------------------------------------------------------------------------------
void setup() 
{
  char buffer[21];                               // buffer for max 20 char display
  Serial.begin(115200);
  
  //--- init Button
  myBtn.begin();                                 // see https://www.thingiverse.com/thing:3641379

  // u8g setup
  u8g.begin() ;
  u8g.setFont(u8g2_font_helvR14_tf);             // 'r' = reduced (or 'n' = numeric) font only for size
  //u8g.setFont(u8g2_font_profont15_tf);         // 'r' = reduced (or 'n' = numeric) font only for size

  strncpy(buffer, "uA meter v", sizeof(buffer));
  strcat(buffer, FIRMWARE_VER);
  LCD_banner(buffer);
  Serial.print("uA meter v"); Serial.println(FIRMWARE_VER);
  Serial.print("Compiled: "); Serial.println(__TIMESTAMP__);
  delay(2000);
  
  // HX711_dout    - pin #A1
  // HX711_sck     - pin #A0
  // if parameter "gain" is ommited; the default value 128 is used by the library
  // 64 & 128 is used for port A ; 32 is used for port B
  scale.begin(HX711_dout, HX711_sck, 128);       // set port based on state of selection, see https://www.thingiverse.com/thing:3641379, not commented out

  LCD_banner("Initializing");
  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));        // print the average of 20 raw readings from the ADC
  
  getOffset();                                   // get the offsets (drift values)
  scale.set_offset(offsetChannelA) ;             // set it for measured channel
  scale.set_scale(calibrationFactorA);           // this value is obtained by calibrating with known value; see the README for details
  
  Serial.print("read: \t\t");
  Serial.println(scale.read());                  // print a raw reading from the ADC
  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(10));        // print the average of 20 readings from the ADC
  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));            // print the average of 5 readings from the ADC minus the tare weight, set with tare()
  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 3);         // print the average of 5 readings from the ADC minus tare weight, divided by scale
  Serial.println("Readings:");
}
//- -----------------------------------------------------------------------------------------------------------------------

//- -----------------------------------------------------------------------------------------------------------------------
// loop
//- -----------------------------------------------------------------------------------------------------------------------
void loop() 
{
  enum state_t {idleState, waitForRelease};      // define possible states
  static state_t state = idleState;  
  //Serial.print("one reading:\t");
  //Serial.print(scale.get_units(), 1);
  //Serial.print("\t| average:\t");
  //Serial.println(scale.get_units(30), 3);
  myBtn.read();                                  // read button state
  switch (state){
    case idleState:                              // nothing
      if (myBtn.wasReleased()){                  // button released = silencePeriod
        LCD_banner("Offset");
        getOffset();                             // get the offsets 
        state = idleState;                       // set silence period
      }
      if (myBtn.pressedFor(1000)){               // long press changes channel, need to wait for release 
        // change channel and wait release
        switchMode();
        state = waitForRelease;
      }
    break;
    case waitForRelease:
      if (myBtn.wasReleased()){                  // button released return to idle
        state = idleState;                       // changed break to end (PeMue)
      }
    break;
  }
  // get ADC readings dependent on setting: read A, B or A & B
  // only A reads has average buffer when A&B mode is selected
  if (convertMode == channelA){
    scale.set_gain(128);
    scale.set_offset(offsetChannelA) ;
    scale.set_scale(calibrationFactorA );        // set division to A value and set mode to A
    lastReading = scale.get_units(32);           // get value (average 32 readings)corrected with scaling
    netReadings[netReadingPointer] = lastReading;  // store readings in averagebuffer
    netReadingPointer = (++netReadingPointer) % netReadingsSize; // increment and wrap
    LCD_local_display();
  } else if (convertMode == channelB){
    scale.set_gain(32);
    scale.set_offset(offsetChannelB);
    scale.set_scale(calibrationFactorB);         // set division to B value and set mode to B
    lastReading = scale.get_units(32);           // get value (average 32 readings)corrected with scaling
    netReadings[netReadingPointer] = lastReading;  // store readings in averagebuffer
    netReadingPointer = (++netReadingPointer) % netReadingsSize; // increment and wrap
    LCD_local_display();
  } else if (convertMode == channelAB){          // if both channels average 128 readings iso 32 (no buffer)
    scale.set_gain(128);
    scale.set_offset(offsetChannelA);
    scale.set_scale(calibrationFactorA);         // set division to A value and set mode to A
    lastReading = scale.get_units(32);           // get value (average 128readings)corrected with scaling
    scale.set_gain(32);
    scale.set_offset(offsetChannelB) ;
    scale.set_scale(calibrationFactorB);         // set division to A value and set mode to A
    lastReadingB = scale.get_units(32);          // get value (average 128readings)corrected with scaling
    LCD_local_displayAB();
  }
  //scale.power_down();                          // put the ADC in sleep mode
  //delay(500);
  //scale.power_up();
  //delay(100);
}
//- -----------------------------------------------------------------------------------------------------------------------

//- -----------------------------------------------------------------------------------------------------------------------
// LCD_banner
//- -----------------------------------------------------------------------------------------------------------------------
void LCD_banner(const char *s)
{
/* prints all avaiable variables on LCD display with units
   input: all "last" variables
*/
  u8g.firstPage();
  do {
    int strWidth = u8g.getStrWidth(s);           // get the length of the string to determine print position
    u8g.drawStr((128- strWidth)/2, 40, s );      // print right aligned 
  } while (u8g.nextPage());
}
//- -----------------------------------------------------------------------------------------------------------------------

//- -----------------------------------------------------------------------------------------------------------------------
// LCD_local_display
//- -----------------------------------------------------------------------------------------------------------------------
void LCD_local_display(void)
{
/* prints all avaiable variables on LCD display with units
   input: all "last" variables
*/
  char buf[21];                                  // buffer for max 20 char display
  char lastNetBuf[14];
  dtostrf(lastReading, 10, 2, lastNetBuf);       // convert real to char
  char averageNetBuf[14];
  dtostrf(netReadingsAverage(), 10, 2, averageNetBuf);     // convert real to char
  char spreadNetBuf[14];
  dtostrf(netReadingsSpread(), 10, 2, spreadNetBuf);       // convert real to char
  Serial.print("Average: \t") ; Serial.print(netReadingsAverage());
  Serial.print("\tSpread: \t") ; Serial.println(netReadingsSpread());

  u8g.firstPage();
  do {
    snprintf(buf, sizeof buf, "Current %s", (convertMode==channelB)?"B":"A"); // header
    int strWidth = u8g.getStrWidth(buf) ;        // length of the string to determine print position
    u8g.drawStr((128- strWidth)/2, 14, buf );    // print middle aligned 
    u8g.drawStr(0,31,"I") ;                      // current
    snprintf(buf, sizeof buf, "%10s\xB5\A", lastNetBuf);
    strWidth = u8g.getStrWidth(buf) ;            // length of the string to determine print position
    u8g.drawStr((128- strWidth), 31, buf );      // print right aligned 
    u8g.drawStr(0,47,"avg") ;                    // average current
    snprintf(buf, sizeof buf, "%10s\xB5\A", averageNetBuf);
    strWidth = u8g.getStrWidth(buf) ;            // get the length of the string to determine print position
    u8g.drawStr((128- strWidth), 47, buf );      // print right aligned 
    u8g.drawStr(0,63,"d\xB1") ;                  // delta +/-
    snprintf(buf, sizeof buf, "%10s\xB5\A", spreadNetBuf);
    strWidth = u8g.getStrWidth(buf) ;            // get the length of the string to determine print position
    u8g.drawStr((128- strWidth), 63, buf );      // print right aligned 
  } while (u8g.nextPage());
}
//- -----------------------------------------------------------------------------------------------------------------------

//- -----------------------------------------------------------------------------------------------------------------------
// LCD_local_display_AB
//- -----------------------------------------------------------------------------------------------------------------------
void LCD_local_displayAB(void)
{
/* prints A & B channel on LCD display with units
   input: all "last" variables
*/
  char buf[21];                                  // buffer for max 20 char display
  char lastNetBuf[14];
  dtostrf(lastReading, 10, 2, lastNetBuf);       // convert real to char
  char lastNetBufB[14];
  dtostrf(lastReadingB, 10, 2, lastNetBufB);     // convert real to char
  char lastNetBufAB[14];
  dtostrf(lastReading +lastReadingB, 10, 2, lastNetBufAB);  // convert real to char for added values
  u8g.firstPage();
  do {
    snprintf(buf, sizeof buf, "Current A+B");    // header
    int strWidth = u8g.getStrWidth(buf);         // length of the string to determine print position
    u8g.drawStr((128- strWidth)/2, 14, buf );    // print middle aligned 
    u8g.drawStr(0,31,"IA");                      // current A
    snprintf(buf, sizeof buf, "%10s\xB5\A", lastNetBuf);
    strWidth = u8g.getStrWidth(buf);             // length of the string to determine print position
    u8g.drawStr((128- strWidth), 31, buf );      // print right aligned 
    u8g.drawStr(0,47,"IB");                      // current B
    snprintf(buf, sizeof buf, "%10s\xB5\A", lastNetBufB);
    strWidth = u8g.getStrWidth(buf) ;            // length of the string to determine print position
    u8g.drawStr((128- strWidth), 47, buf );      // print right aligned 
    u8g.drawStr(0,63,"A+B");                     // current A + B
    snprintf(buf, sizeof buf, "%10s\xB5\A", lastNetBufAB);
    strWidth = u8g.getStrWidth(buf) ;            // length of the string to determine print position
    u8g.drawStr((128- strWidth), 63, buf );      // print right aligned 
  } while (u8g.nextPage());
}
//- -----------------------------------------------------------------------------------------------------------------------

//- -----------------------------------------------------------------------------------------------------------------------
// netReadingsAverage
//- -----------------------------------------------------------------------------------------------------------------------
// calculate average of net readings
double netReadingsAverage()
{
  double sum = 0;
  for (byte i = 0; i < netReadingsSize; i++){
    sum += netReadings[ i ];
  }
  return sum / netReadingsSize;
}
//- -----------------------------------------------------------------------------------------------------------------------

//- -----------------------------------------------------------------------------------------------------------------------
// netReadingsSpread
//- -----------------------------------------------------------------------------------------------------------------------
// calculate spread of net readings (+/-)
double netReadingsSpread()
{
  double minReading = netReadings[0];
  double maxReading = minReading ;
  for (byte i = 1; i < netReadingsSize; i++){
    if (minReading > netReadings[ i ]){
      minReading = netReadings[i] ;
    }
    if (maxReading < netReadings[ i ]){
      maxReading = netReadings[i] ; 
    }
  }
  return (maxReading - minReading)/2;
}
//- -----------------------------------------------------------------------------------------------------------------------

//- -----------------------------------------------------------------------------------------------------------------------
// switchMode
//- -----------------------------------------------------------------------------------------------------------------------
// switch the mode
void switchMode()
{
  if (convertMode == channelA){
    convertMode = channelB;
  } else if (convertMode == channelB){
    convertMode = channelAB;
  } else {
    convertMode = channelA;
  }
}
//- -----------------------------------------------------------------------------------------------------------------------

//- -----------------------------------------------------------------------------------------------------------------------
// getOffset
//- -----------------------------------------------------------------------------------------------------------------------
// assuming both channels are shorted, calculate the offset values for channel A and B
double getOffset()
{
  scale.set_gain(128);                           // get channel A
  offsetChannelA = scale.read_average(32);       // average 512 readings for offset
  Serial.print("Offset A: \t"); 
  Serial.println(offsetChannelA);
  scale.set_gain(32);                            // get channel B
  offsetChannelB = scale.read_average(32);       // average 512 readings for offset
  Serial.print("Offset B: \t"); 
  Serial.println(offsetChannelB);
}
//- -----------------------------------------------------------------------------------------------------------------------
