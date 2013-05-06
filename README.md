# **Ice Tube Clock firmware with GPS & working Auto DST support** #

Latest update May 6, 2013 - William Phelps - wm (at) usa.net 
 
## **Includes the following modifications:** ##

**Updates in v130505wm**  
06may13 - option to flash dp if no GPS signal ("secs=fullgps")  
06may13 - clean up all warning messages
 
**Updates in v130505wm**  
 05may13 - add GPS 2 message test
 Note: 2 GPRMC messages in a row must have same year, month, day, hour, & minute 
 This prevents a garbled GPS messages from setting the clock to the wrong time or date

 **Updates in v130406wm**  
 06apr13 - fix error in Auto DST for southern hemisphere  

**Updates in v121105wm**  
 05nov12 - fix bugs in Auto DST code  
 Change Makefile to support atmega328p  
 **NOTE**: This version will probably not fit on a 168p with all features enabled!

**Updates in v121012wm**  
 12oct12 - fix set volume low/high  
 06oct12 - fix Auto DST in southern hemisphere  
 27sep12 - add support for 9600 bps gps  
 28aug12 - fix check for space in gps buffer (string null term)
 
**Changes to run on Atmega328p**  
Interrupt names

**Rewrite of GPS code**
Better error checking, including CRC check  
GPS error counters visible in menu

**Improvements to GPS support**  
GPS Enable/Disable in Menu  
GPS Lat & Long displayed in Menu

**Rewrite of Auto DST so it works with GPS**  
DST update now works regardless of when clock is powered on  
update occurs once per minute if clock is powered up "cold" with DST active  
DST change will occur next time Seconds = "00"  
DST Rules in Menu (read code before changing!)

**Changes to Auto Dim feature**  
lo and hi brightness values in menu  
use 20k ohm resistor  
menu option to show ADC and Brightness values (press select button to update display)

**Added Progressive Alarm feature**
alarm starts out with single beep at long(ish) interval  
more beeps, shorter interval as time elapses

**Includes Drift Correction & Seconds Dial** features from jsgf (by request)

**Added Last Digit Brightness Boost (LDBB)**  
for vfd tubes with dim right most digit

**Added #IFDEF for GPS support (user request)**

 