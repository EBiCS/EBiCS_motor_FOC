Fork of EBiCS firmware for Lishui devices. Ported to Xaiomi M365 controller. 


Note:  This is WiP, use at your own risk. 
Note:  Makes no attempt to implement M365 protocols between BMS/BLE controllers, currently targets analog throttle soldered to CPU pin 11(PA1)
Note:  On my controller I had to short the Hall A input resistor, some pullup current is very high. Unsure of cause. 

Currently can do hall autodetection, and then run in sinusoidal mode with throttle controlled voltage. 
FoC and current limiting in general is not working right yet. 

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

# Remote control from ESP32
You can use analog throttle and brake and a ESP32 to remote control the SmartESC :
https://github.com/Koxx3/SmartESC_ESP32_serial_control/tree/SmarESC_V3