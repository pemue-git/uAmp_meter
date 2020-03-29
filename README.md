# uAmp_Meter

Firmware for the uAmp meter with HX711 chip.

## Changelog firmware
### v0.1 (PeMue)
This firmware is based on the [original firmware](https://www.openhardware.io/dl/58ff592144f656d179dca241/code/AWI_uA_meter.ino) and the [proposed changes](https://www.thingiverse.com/thing:3641379) (some are not working). For a firmware with more debug output see [here](https://forum.fhem.de/index.php?action=dlattach;topic=104466.0;attach=132982).
- added HX11 specifications (juergs)
- added firmware version and compile time
- changed sequence of functions (setup and loop to the end)
- changed getOffset to void

## Flashing the firmware
1. Install the Arduino IDE and chose Arduino nano (Atmega 328P) as board
2. Open, compile and upload the *uAmp_meter.ino* sketch to your Arduino nano
3. Press button short and apply known current to channel A and read the value
4. Press button long (to get to channel B), press button short and repeat step 3.
5. Calculate calibration factors for channel A and B and put into your sketch and repeat step 1.
