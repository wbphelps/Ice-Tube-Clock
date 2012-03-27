# **Ice Tube Clock firmware with GPS & working Auto DST support** #

Latest update March 2012 - William Phelps - wm (at) usa.net 
 
## **Includes the following modifications:** ##
 
**Improvements to GPS support**  
GPS Enable/Disabble in Menu  
GPS Lat & Long displayed in Menu

**Rewrite of Auto DST so it works with GPS**  
DST update now works regardless of when clock is powered on  
update occurs once per minute if clock is powered up "cold" with DST active  
DST change will occur next time Seconds = "00"  
DST Rules in Menu (read code before changing!)

**Changes to Auto Dim feature**  
lo and hi brightness values in menu  
use 20k ohm resistor  
menu option to show ADC and Brightness values  
press select button to update

**Added Progressive Alarm feature**
alarm starts out with single beep at long(ish) interval  
more beeps, shorter interval as time elapses

**Includes Drift Correction & Seconds Dial** features from jsgf (by request)

**Added Last Digit Brightness Boost (LDBB)**  
for vfd tubes with dim right most digit

**Added #IFDEF for GPS support (user request)**

 