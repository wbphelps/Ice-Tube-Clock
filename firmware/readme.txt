Clock has 3 buttons: Menu, Select, Set

1) MENU - cycle thru Menu items
2) SELECT - select (next) Menu item
3) SET - adjust/store setting

Clock has the following Menu items:
(if all features are enabled)

) ICE TUBE - press SELECT to see firmware version
) SET ALAR - set alarm time
) SET TIME - set clock time
) SET DATE - set clock date
) SET ADIM - enable Auto Dimming feature
) SET BRIT - set Brightness level or levels
) SET DST  - set DST feature Off/On/Auto
) SET DRFT - drift correction
) SET GPS  - GPS feature: Off, 48, 96
) SET LDBB - last digit brightness adjust
) SET VOL  - set beeper Volume Low/High
) SET REGN - set display "Region" 12/24 hour
) SET RULE - set DST rules (do not adjust)
) SET SECS - seconds display mode
) SET SNOZ - set Snooze time
) SET ZONE - set time zone, 0 for GMT, negative for West, Eastern US is -5

ldbb is "last digit brightness boost"
drft is drift correction, compensates for small errors in crystal frequency
not needed if GPS is connected
secs is "seconds display" - try it, there are several choices. "full" is 
'hh mm ss'. "fullgps" shows gps signal status with a decimal point on the 
right seconds digit.
rules should be "3, 1, 2, 2, 11, 1, 1, 2, 1" (1st sunday in march, etc)

Always press Set button one more time after any adjustments - this
will store the new value(s) in the clocks eeprom memory.

If GPS feature is On and GPS has signal (red LED not blinking) and you
adjust time or date, clock will set itself again within a couple of
seconds.  To demonstrate GPS feature, turn it off, adjust time/date,
then turn GPS feature back on and watch the clock set itself.

GPS can now be set to "48" for 4800 baud, or "96" for 9600 baud

Auto DST will adjust time automatically when DST starts/ends based on
Rules.  Rules give start, end, offset of DST: Start(month, dotw, n,
hour), End(month, dotw, n, hour), Offset