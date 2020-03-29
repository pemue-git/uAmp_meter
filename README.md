# uAmp_Meter

Firmware for the uAmp meter with HX711 chip.

## Changelog firmware
### v0.1 (PeMue)
This firmware is based on the original firmware and the proposed changes (some are not working).
- added HX11 specifications (juergs)
- added firmware version and compile time
- changed sequence of functions (setup and loop to the end)
- changed getOffset to void

## Flashing the firmware
1. Install the Arduino IDE and chose Arduino nano (Atmega 328P) as board
2. Open, compile and upload the *uAmp_meter.ino* sketch to your Arduino nano
3. Press button short and apply known current to channel A and read the value
4. Press button long (to get to channel B), press button short and repeat step 3.
5. Calculate calibration factors for channel A and B and put into your sketch and repeat step 1
