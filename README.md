# UCT SHARC Buoy: Snow Depth Sensor (SDS) System - Code Design 
by Eunseong Kim (KMXEUN004)
#
### This GitHub respiratory contains the overall demo code design developed during the "UCT SHARC Buoy: Snow Depth Sensor (SDS) System Design" project. The code can be found under the folder "Final_SnowDepthSensor", named "Final_SnowDepthSensor.ino". 
#
* All code design was done using Arduino IDE.
* Before uploading this code onto an Arduino microcontroller (Arduino Pro Mini 3.3 V as per designed), the overall circuit needs to be built (see section 6.1.2).
* The overall flow diagram of this code deesign can be found in section 6.2.1.
#
<br />


The code design contains libraries which needs to be downloaded in order to use the code design. The following libraries and the associated links are listed below:
* **"VL53L1X.h"**, library used for communication with the VL53L1X sensor. ("Continuous", https://github.com/pololu/vl53l1x-arduino), (Pololu, et al., 2021).
* **"RTClib.h"**, library for controlling the DS3231 RTC. (“DS3231_alarm”, https://github.com/adafruit/RTClib), (Adafruit: MIT License, n.d.).
* **"IridiumSBD.h"**, library for communucatoin between Arduino and Iridium modems.(https://github.com/mikalhart/IridiumSBD/blob/master/src/IridiumSBD.h), (Hart, 2019).

* **"LowPower.h"** and **"avr/sleep.h"**, Arduino standard libraries for the sleep function.


There are several manual inputs required, which are as follows:
* **Distance to Zero Level** in mm (manually measured perpendicular distance from the VL53L1X sensor to the ice surface, on which the buoy is installed).
* **Compensation for Light Penetration Depth** in mm (expected to be less than 20 mm, however, further testing needs to be performed to fully understand the phenomenon).
* **Measurement duration** in ms needs to be configured, considering power efficiency, the value was set to 10000 ms (10 seconds).
* **Window Size** for the Moving Average Filter was chosen to be 20 after testing and simulation (see sectiom 7.2).
* **Inter-measurement period** (the duration of passive phase) designed to be 6 hours for power efficient design. This value may be changed if necessary.
<br />

**The code is yet a demo design, whereby further improvements are necessary. For example, UART communication with RockBlock9603 was designed based on theory, which needs to be validated thorugh actual communicaiton with the Iridium Modem.**
