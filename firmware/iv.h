/***************************************************************************
 Ice Tube Clock with GPS firmware November 15, 2011
 (c) 2011 William B Phelps
 
 17Nov11 - GPS vs Alarm bug
 14Nov11 - progressive alarm beeping
 07Nov11 - fix bug, change DST setting to Off, On, Auto
 09May11 - capture & display GPS Lat and Long
 27Apr11 - set DST offset on boot & when rules are changed
 26Apr11 - add DST Rules setting to menu, save in EE
 21Apr11 - add Brightness low and high limits to menu, make Auto Bright separate option 
 20Apr11 - rewrite DST code so it works all the time and with GPS
 18Apr11 - Add option to disable/enable GPS
 
 Ice Tube Clock firmware August 13, 2009
 (c) 2009 Limor Fried / Adafruit Industries
 Ice Tube Clock with GPS firmware July 22, 2010
 (c) 2010 Limor Fried / Adafruit Industries
 GPS Capability added by Devlin Thyne
 Modifications by Len Popp
 Original auto-dimmer mod by Dave Parker
 Button interrupt fix by caitsith2
 Daylight Saving Time code
 Testmode feature by caitsith2

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
****************************************************************************/

// Optional Features - #define these or not as desired.
// Auto-dimmer - requires a photocell hooked up to the hax0r port
#define FEATURE_AUTODIM
// Display digit "9" in the usual way (instead of the default with no bottom segment)
#define FEATURE_9
// Enables Daylight Saving Time.
#define FEATURE_WmDST
#ifdef FEATURE_WmDST
  #define DST_OFF 0
	#define DST_ON  1
  #define DST_AUTO 33  // value chosen to help detect random EE value
	#define DST_NO  0 
	#define DST_YES 1  // Clock has been updated for DST Change
#endif

#define GPS_OFF 0
#define GPS_ON  1

// Allows Testing of the Hardware.  If FEATURE_AUTODIM is enabled, it will
// include testing of the hax0r port photocell.
//#define FEATURE_TESTMODE

// Allows setting how long a snooze period lasts for. (10 minutes default.)
// LadyAda implemented this, then commented it out.  I (caitsith2) just turned 
// it into defines. 
#define FEATURE_SETSNOOZE

#define halt(x)  while (1)

#define DEBUG 1
#define DEBUGP(x)  if (DEBUG) {putstring_nl(x);}

//The year the clock was programmed, used for error checking
#define PROGRAMMING_YEAR 11

#define REGION_US 0
#define REGION_EU 1

// date format
#define DATE 0  // mm-dd-yy
#define DAY 1   // thur jan 1

// String buffer size:
#define GPSBUFFERSIZE 128

#define DISPLAYSIZE 9

#define MAXSNOOZE 600 // 10 minutes
#define INACTIVITYTIMEOUT 10 // how many seconds we will wait before turning off menus

#define BRITE_MAX 90
#define BRITE_MIN 30
#define BRITE_INCREMENT 5
#define AUTODIM_OFF 0
#define AUTODIM_ON 1
#define AUTODIM_MIN 5  // much lower minimum for autodim

#define PHOTOCELL_DARK 1010
#define PHOTOCELL_LIGHT 100  // wbp (was 500)

#define BEEP_8KHZ 5
#define BEEP_4KHZ 10
#define BEEP_2KHZ 20
#define BEEP_1KHZ 40

#define EE_INIT 0
#define EE_YEAR 1
#define EE_MONTH 2
#define EE_DAY 3
#define EE_HOUR 4
#define EE_MIN 5
#define EE_SEC 6
#define EE_ALARM_HOUR 7 
#define EE_ALARM_MIN 8
#define EE_BRIGHT 9  
#define EE_VOLUME 10
#define EE_REGION 11
#define EE_SNOOZE 12

#define EE_ZONE_HOUR 13
#define EE_ZONE_MIN 14

#define EE_DSTMODE 15
#define EE_GPSENABLE 16

#define EE_AUTODIM 17
#define EE_AUTODIMLO 18
#define EE_AUTODIMHI 19

#define EE_DSTRULE0 20
#define EE_DSTRULE1 21
#define EE_DSTRULE2 22
#define EE_DSTRULE3 23
#define EE_DSTRULE4 24
#define EE_DSTRULE5 25
#define EE_DSTRULE6 26
#define EE_DSTRULE7 27
#define EE_DSTRULE8 28
#define EE_DSTOFFSET 29

void delay(uint16_t delay);
void _delay_ms(uint32_t __ms);
void _delay_loop_2(uint16_t __count);

void (*app_start)(void) = 0x0000;

// Add Initialization routine defines here.
void init_eeprom(void);
void clock_init(void);
void initbuttons(void);
void boost_init(uint8_t pwm);
void vfd_init(void);
void set_vfd_brightness(uint8_t brightness);
void speaker_init(void);
#ifdef FEATURE_AUTODIM
void dimmer_init(void);
void dimmer_update(void);
#endif

char get_number(char b);
char get_alpha(char b);

void display_time(uint8_t h, uint8_t m, uint8_t s);
void display_date(uint8_t style);
void display_str(char *s);
void display_alarm(uint8_t h, uint8_t m);
void display_timezone(int8_t h, uint8_t m);
void display_brightness(int brightness);
void display_autodim(int autodim);
void display_dstrule(uint8_t i);

void set_time(void);
void set_alarm(void);
void start_alarm(void);
void set_date(void);
void show_about(void);
void set_brightness(void);
#ifdef FEATURE_AUTODIM
void set_autobrightness(void);
void set_autodim(void);
#endif
void set_volume(void);
void set_region(void);
void set_snoozetime(void); // not activated by default
#ifdef FEATURE_WmDST
void set_dstmode(void);
void set_dstrules(void);
#endif
void set_gpsenable(void);  // wbp
void show_gpslat(void);  // wbp
void show_gpslng(void);  // wbp
void set_test(void);

//Checks the alarm against the passed time.
void check_alarm(uint8_t h, uint8_t m, uint8_t s);
//Fixes the time variables whenever time is changed
void fix_time(void);
//Set the time zone:
void set_timezone(void);
#ifdef FEATURE_WmDST
long Seconds(uint8_t yr, uint8_t mo, uint8_t da, uint8_t h, uint8_t m, uint8_t s);
void setDSToffset(uint8_t rules[]);
void save_dstrules(uint8_t mods[]);
#endif

void beep(uint16_t freq, uint8_t times);
void beep_ms(uint16_t freq, uint8_t times, uint16_t ms);
void tick(void);

uint8_t leapyear(uint16_t y);
uint8_t dotw(uint8_t year, uint8_t month, uint8_t day);
void setalarmstate(void);

//void setdisplay(uint8_t digit, uint8_t segments);
void setdisplay(uint8_t digit);
void vfd_send(uint32_t d);
void spi_xfer(uint8_t c);

//GPS serial data handling functions:
uint8_t gpsDataReady(void);
void getGPSdata(void);
void getGPStime(void);
void setgpstime(char* str);
void setgpsdate(char* str);

// displaymode (Menu items)
enum dispmodes {
	SHOW_TIME,
	SHOW_ABOUT,
	SET_ALARM,
	SET_TIME,
	SET_DATE,
	SET_ZONE,
	SET_GPSENABLE,
#ifdef FEATURE_AUTODIM
	SET_AUTODIM,
#endif
	SET_BRIGHTNESS,
	SET_VOLUME,
	SET_REGION,
#ifdef FEATURE_SETSNOOZE
	SET_SNOOZETIME,
#endif
#ifdef FEATURE_WmDST
	SET_DSTMODE,
	SET_DSTRULES,
#endif
#ifdef FEATURE_TESTMODE
	TESTMODE,
#endif
	N_MODES		// Number of Menu "modes", must be last in this enum
};

#define NONE 99
#define SHOW_SNOOZE 98  // not menu item, used to block time display

// sub-mode settings
#define SHOW_MENU 0
// alarm/time
#define SET_HOUR 1
#define SET_MIN 2
#define SET_SEC 3
// date
#define SET_MONTH 1
#define SET_DAY 2
#define SET_YEAR 3
//brightness (if not auto_bright)
#define SET_BRITE 1
//brightness (if auto_bright)
#define SET_AUTODIMLO 1
#define SET_AUTODIMHI 2
//autodim
//#define SET_AUTODIM 1
//volume
#define SET_VOL 1
//region
#define SET_REG 1
//dst
#define SET_DST 1
//gps
#define SET_GPS 1
#define SHOW_GPSLAT1 2
#define SHOW_GPSLAT2 3
#define SHOW_GPSLONG1 4
#define SHOW_GPSLONG2 5

#define BOOST PD6
#define BOOST_DDR DDRD
#define BOOST_PORT PORTD

#define BUTTON1 PD5
#define BUTTON2 PB0
#define BUTTON3 PD4

#define VFDSWITCH PD3
#define VFDSWITCH_DDR DDRD
#define VFDSWITCH_PORT PORTD
#define VFDCLK PB5
#define VFDCLK_DDR DDRB
#define VFDCLK_PORT PORTB
#define VFDDATA PB3
#define VFDDATA_DDR DDRB
#define VFDDATA_PORT PORTB
#define VFDLOAD PC0
#define VFDLOAD_DDR DDRC
#define VFDLOAD_PORT PORTC
#define VFDBLANK PC3
#define VFDBLANK_DDR DDRC
#define VFDBLANK_PORT PORTC

#define ALARM PD2
#define ALARM_DDR DDRD
#define ALARM_PORT PORTD
#define ALARM_PIN PIND

#define SPK1 PB1
#define SPK2 PB2
#define SPK_PORT PORTB
#define SPK_DDR DDRB

#define DIMMER_POWER_PORT PORTC
#define DIMMER_POWER_DDR DDRC
#define DIMMER_POWER_PIN PC5
#define DIMMER_SENSE_PIN MUX2
#define DIMMER_SENSE_PIND ADC4D

#define SEG_A 19
#define SEG_B 17
#define SEG_C 14
#define SEG_D 13
#define SEG_E 15
#define SEG_F 18
#define SEG_G 16
#define SEG_H 11

#define DIG_1 4
#define DIG_2 12
#define DIG_3 5
#define DIG_4 10
#define DIG_5 6
#define DIG_6 9
#define DIG_7 8
#define DIG_8 7
#define DIG_9 3

#define nop asm("nop")
