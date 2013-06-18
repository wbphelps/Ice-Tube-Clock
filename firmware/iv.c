/***************************************************************************
 Ice Tube Clock with GPS & Auto DST firmware Nov 6, 2011
 (c) 2009 Limor Fried / Adafruit Industries
 (c) 2013 William B Phelps

 17jun13 - revert to old eeprom method
 16jun13 - if dst mode not Auto, don't call setdstoffset when rules changed or time set
 10may13 - change EE read/write to use ee_update_byte & compiler ee mapping
 09may13 - fix silly bug in auto dst
 06may13 - option to flash dp if no GPS signal
 06may13 - clean up all warning messages
 05may13 - add gps 2 message test
 06apr13 - fix error in Auto DST for southern hemisphere
 05nov12 - fix bugs in Auto DST code
 12oct12 - fix set volume low/high
 06oct12 - fix Auto DST in southern hemisphere
 27sep12 - add support for 9600 bps gps
 28aug12 - fix check for space in gps buffer (string null term)
 01aug12 - modify to work on atmega328 as well as 168
 30jul12 - rewrite GPS receive logic
 27jul12 - if GPS on & no signal, show message
 25jul12 - add GPS checksum check!
 25jul12 - stop alarm going off at midnight
 26Jan12 - drift_correction wasn't working
 12Jan12 - ifdef for GPS support, show adc level
 06Jan12 - seconds dial mode
 27Dec11 - add menu option for last digit brightness boost (ldbb)
 22Dec11 - add Drift Correction feature
 18Nov11 - progressive alarm feature, alarm startup bug
 17Nov11 - GPS vs Alarm bug
 07Nov11 - fix bug, change DST setting to Off, On, Auto
 14May11 - clean up "abrt" vs "adim" - which is better?
 09May11 - capture & display GPS Lat and Long
 27Apr11 - set DST offset on boot & when rules are changed
 26Apr11 - add DST Rules setting to menu, save in EE
 21Apr11 - add Brightness low and high limits to menu, make Auto Bright separate option 
 20Apr11 - rewrite DST code so it works all the time and with GPS
 18Apr11 - Add option to disable/enable GPS
 
 Ice Tube Clock with GPS firmware March 2012
 (c) 2010 Limor Fried / Adafruit Industries
 GPS Capability added by Devlin Thyne
 Ice Tube Clock firmware August 13, 2009
 (c) 2009 Limor Fried / Adafruit Industries
 Modifications by Len Popp
 Original auto-dimmer mod by Dave Parker
 Button interrupt fix by caitsith2
 Daylight Saving Time code
 Testmode feature by caitsith2

Note: In this clock, the microcontroller runs off its internal 8 MHz oscillator. 
The external clock crystal is used to provide interrupts 32768 times per second, which are counted and used to tell the time.

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

#include <avr/io.h>      
#include <string.h>
#include <avr/interrupt.h>   // Interrupts and timers
//#include <util/delay.h>      // Blocking delay functions
#include <avr/pgmspace.h>    // So we can store the 'font table' in ROM
#include <avr/eeprom.h>      // Date/time/pref backup in permanent EEPROM
#include <avr/wdt.h>     // Watchdog timer to repair lockups
#include <stdlib.h>

#include "iv.h"
#include "util.h"
#include "fonttable.h"

#ifdef FEATURE_DRIFTCORR
#define DRIFT_BASELINE	127  // OCR2A Baseline (uncorrected)
static int8_t drift_corr = 0;  /* Drift correction applied each hour */
#endif

char version[8] = "130617wm";  // program timestamp/version

uint8_t region = REGION_US;

// These variables store the current time.
volatile int8_t time_s, time_m, time_h;
// ... and current date
volatile uint8_t date_m=03, date_d=14, date_y = 47;

// how loud is the speaker supposed to be?
volatile uint8_t volume;

volatile uint8_t bright_level;  // brightness set by user if autodim is off
#ifdef FEATURE_AUTODIM
volatile uint8_t autodim_lo;
volatile uint8_t autodim_hi;
volatile uint8_t autodim = 0;
volatile uint16_t dimmer_adc;  // last result from ADC
volatile uint16_t dimmer_lvl;  // current dimmer level
#endif

/* Bit correspondence
 *     -80-      -80-
 *    |    |    |    |
 *   04    40  04    40
 *    |    |    |    |
 *     -02-      -02-
 *    |    |    |    |
 *   08    20  08    20
 *    |    |    |    |
 *     -10- o01  -10- o01
 */
#ifdef FEATURE_SECSMODE
volatile uint8_t secsmode = 0;
uint8_t secsdiv = 0;
uint8_t secscnt = 0;
#define SECS_DIVIDER 150  // set freq to 1/6 second.  secsmdiv is incremented at 900 hz

const uint16_t dialsegs1[] PROGMEM =  // inner segment dance, by Larry Frank
  {0x620E, 0x200E, 0x400E, 0x600C, 0x6204, 0x6208};  // 6 steps
PGM_P const dialsegs1_p PROGMEM = (char*)dialsegs1;
const uint16_t dialsegs2[] PROGMEM =  // outer segments only
  {0x8080, 0x0080, 0x0040, 0x0020, 0x0010, 0x1010, 0x1000, 0x0800, 0x0400, 0x8000};  // 10 steps
//const uint16_t dialsegs2[] PROGMEM =  // outer segments only
//  {0x9C70, 0x9CB0, 0x9CD0, 0x9CE0, 0x8CF0, 0x94F0, 0x98F0, 0x1CF0};  // 8 steps
PGM_P const dialsegs2_p PROGMEM = (char*)dialsegs2;
//const uint16_t dialsegs3[] PROGMEM =  // radial pattern, outer segment cups
//	{0xC084, 0x00C4, 0x00C2, 0x0062, 0x0032, 0x0038, 0x3018, 0x3800, 0x1A00, 0x0E00, 0x8600, 0xC400};  // 12 steps
const uint16_t dialsegs3[] PROGMEM =  // snake pattern, 12 steps
	{0x80C0, 0x00C2, 0x0242, 0x0A02, 0x1A00, 0x1810, 0x1030, 0x0032, 0x0222, 0x0602, 0x8600, 0x8480};  
PGM_P const dialsegs3_p PROGMEM = (char*)dialsegs3;
const uint16_t dialsegs4[] PROGMEM =  // inner segment radial pattern, fast
  {0x0004, 0x0002, 0x0008, 0x2000, 0x0200, 0x4000};  // 6 steps
PGM_P const dialsegs4_p PROGMEM = (char*)dialsegs4;

 #endif

// whether the alarm is on, going off, and alarm time
volatile uint8_t alarm_on, alarming, alarm_h, alarm_m;
volatile uint16_t alarm_timer, alarm_count, alarm_cycle, beep_count, beep_cycle;

// what is being displayed on the screen? (eg time, date, menu...)
volatile uint8_t displaymode;

// are we in low power sleep mode?
volatile uint8_t sleepmode = 0;

volatile uint8_t timeknown = 0;        // MEME
volatile uint8_t dateknown = 0;
volatile uint8_t restored = 0;

#ifdef FEATURE_WmDST
volatile uint8_t dst_mode;
volatile uint8_t dst_offset = 0;  // current DST offset, also used by gpssettime
volatile uint8_t dst_update = 0;  // DST Update allowed now? (Reset at midnight)
uint8_t dst_rules[9]={3,1,2,2,11,1,1,2,1};   // initial values from US DST rules as of 2011
// DST Rules: Start(month, dotw, n, hour), End(month, dotw, n, hour), Offset
// DOTW is Day of the Week, 1=Sunday, 7=Saturday
// N is which occurrence of DOTW
// Current US Rules:  March, Sunday, 2nd, 2am, November, Sunday, 1st, 2 am, 1 hour
const uint8_t dst_rules_lo[]={1,1,1,0,1,1,1,0,0};  // low limit
const uint8_t dst_rules_hi[]={12,5,7,23,12,5,7,23,1};  // high limit
static const uint8_t mDays[]={31,28,31,30,31,30,31,31,30,31,30,31}; // number of days in each month
static const uint16_t tmDays[]={0,31,59,90,120,151,181,212,243,273,304,334}; // total number of days by month
#endif

#ifdef FEATURE_GPS
// String buffer for processing GPS data:
char gpsBuffer[GPSBUFFERSIZE];
volatile uint8_t gpsEnabled = 0;
#define gpsTimeout1 5  // 5 seconds until we turn off the gps signal dp
#define gpsTimeout2 15  // 15 seconds until we display the "no gps" message or start flashing the dp
uint8_t gpsTimeout = 0;  // how long since we received valid GPS data? (in seconds)
char gpsTime[7];
char gpsDate[7];
char gpsFixStat[1];  // fix status
char gpsLat[6];  // ddmmff  (without decimal point)
char gpsLatH[1];  // hemisphere 
char gpsLong[7];  // ddddmmff  (without decimal point)
char gpsLongH[1];  // hemisphere 
char *gpsPtr;
char gpsCheck1, gpsCheck2;
char gpsPrevTime[7] = "000000";
char gpsPrevDate[7] = "000000";
#endif

// Variables for the timezone offset if using GPS.
int8_t intTimeZoneHour = -8;  //PST
uint8_t intTimeZoneMin = 0;

// Our display buffer, which is updated to show the time/date/etc
// and is multiplexed onto the tube
uint8_t display[DISPLAYSIZE]; // stores segments, not values!
uint8_t currdigit = 0;        // which digit we are currently multiplexing
uint8_t lastdigit = DISPLAYSIZE-1;  // last digit before starting over
#ifdef FEATURE_LDBB
volatile uint8_t ldbb = 0;  // Last Digit Brightness Boost
#endif

// This table allow us to index between what digit we want to light up
// and what the pin number is on the MAX6921 see the .h for values.
// Stored in ROM (PROGMEM) to save RAM
const char digittable[] PROGMEM = {
  DIG_9, DIG_8, DIG_7, DIG_6, DIG_5, DIG_4, DIG_3, DIG_2, DIG_1
};
PGM_P const digittable_p PROGMEM = digittable;

// This table allow us to index between what segment we want to light up
// and what the pin number is on the MAX6921 see the .h for values.
// Stored in ROM (PROGMEM) to save RAM
const char segmenttable[] PROGMEM = {
  SEG_H, SEG_G,  SEG_F,  SEG_E,  SEG_D,  SEG_C,  SEG_B,  SEG_A 
};
PGM_P const segmenttable_p PROGMEM = segmenttable;

// muxdiv and MUX_DIVIDER divides down a high speed interrupt (31.25KHz)  
// down so that we can refresh at about 100Hz (31.25KHz / 300)
// We refresh the entire display at 100Hz so each digit is updated
// 100Hz/DISPLAYSIZE
uint16_t muxdiv = 0;
//#define MUX_DIVIDER (300 / DISPLAYSIZE)  
#define MUX_DIVIDER (312 / DISPLAYSIZE)  // wbp 

// Likewise divides 100Hz down to 1Hz for the alarm beeping
uint16_t alarmdiv = 0;
#define ALARM_DIVIDER 90  // set alarm freq to 1/10 second.  alarmdiv is incremented at 900 hz, not 100! (wbp)

// How long we have been snoozing
uint16_t snoozetimer = 0;

// Settings saved to eeprom
// uint8_t EEMEM EE_INIT = 1;

// uint8_t EEMEM EE_YEAR = 13;
// uint8_t EEMEM EE_MONTH = 6;
// uint8_t EEMEM EE_DAY = 17;  // june 17, 2013
// uint8_t EEMEM EE_HOUR = 0;
// uint8_t EEMEM EE_MIN = 0;
// uint8_t EEMEM EE_SEC = 0;

// uint8_t EEMEM EE_ALARM_HOUR = 7;
// uint8_t EEMEM EE_ALARM_MIN = 0;  // Alarm 07:00
// uint8_t EEMEM EE_SNOOZE = 10;  // default 10 minute Snooze

// uint8_t EEMEM EE_BRIGHT = 65;  //Brightness level = 65
// uint8_t EEMEM EE_VOLUME = 0;  //Volume low

// uint8_t EEMEM EE_REGION = REGION_US;  // 12 hour mode
// uint8_t EEMEM EE_ZONE_HOUR = 4;  // Time Zone hour (GPS) (offset by 12 to make positive)
// uint8_t EEMEM EE_ZONE_MIN = 0;  // Time Zone minute (GPS)
// uint8_t EEMEM EE_DSTMODE = 0;  // No Daylight Saving Time
// uint8_t EEMEM EE_GPSENABLE = 0;  // GPS disabled
// uint8_t EEMEM EE_AUTODIM = 0;  // Autodim disabled
// uint8_t EEMEM EE_AUTODIMLO = 20;  // Auto Brightness Level Low
// uint8_t EEMEM EE_AUTODIMHI = 85;  // Auto Brightness Level High

// uint8_t EEMEM EE_DST_RULE0 = 3;  // DST start month
// uint8_t EEMEM EE_DST_RULE1 = 1;  // DST start dotw
// uint8_t EEMEM EE_DST_RULE2 = 2;  // DST start week
// uint8_t EEMEM EE_DST_RULE3 = 2;  // DST start hour
// uint8_t EEMEM EE_DST_RULE4 = 11; // DST end month
// uint8_t EEMEM EE_DST_RULE5 = 1;  // DST end dotw
// uint8_t EEMEM EE_DST_RULE6 = 1;  // DST end week
// uint8_t EEMEM EE_DST_RULE7 = 2;  // DST end hour
// uint8_t EEMEM EE_DST_RULE8 = 1;  // DST offset

// uint8_t EEMEM EE_DSTOFFSET = 0;  // DST Offset
// uint8_t EEMEM EE_SECSMODE = 0;  // Seconds display mode
// uint8_t EEMEM EE_DRIFTCORR = 0;  // Drift correction
// uint8_t EEMEM EE_LDBB = 0;  // Last Digit Brightness Boost

// uint8_t EEMEM EE_TEST = 42;  // last item...

// We have a non-blocking delay function, milliseconds is updated by
// an interrupt
volatile uint16_t milliseconds = 0;
void delayms(uint16_t ms) {
  sei();
  milliseconds = 0;
  while (milliseconds < ms);
}

// copy of _delay_ms but with uint instead of double
void _delay_ms(uint32_t __ms)
{
	uint16_t __ticks;
	uint32_t __tmp = ((F_CPU) / 4000) * __ms;
	if (__tmp < 1)
		__ticks = 1;
	else if (__tmp > 65535)
	{
		//	__ticks = requested delay in 1/10 ms
		__ticks = (uint16_t) (__ms * 10);
		while(__ticks)
		{
			// wait 1/10 ms
			_delay_loop_2(((F_CPU) / 4000) / 10);
			__ticks --;
		}
		return;
	}
	else
		__ticks = (uint16_t)__tmp;
	_delay_loop_2(__ticks);
}
void _delay_loop_2(uint16_t __count)
{
	__asm__ volatile (
		"1: sbiw %0,1" "\n\t"
		"brne 1b"
		: "=w" (__count)
		: "0" (__count)
	);
}

// When the alarm is going off, pressing a button turns on snooze mode
// this sets the snoozetimer off in MAXSNOOZE seconds - which turns on
// the alarm again
void setsnooze(void) {
#ifdef FEATURE_SETSNOOZE
  snoozetimer = eeprom_read_byte((uint8_t *)EE_SNOOZE);
  snoozetimer *= 60; // convert minutes to seconds
#else
  snoozetimer = MAXSNOOZE;
#endif
//  DEBUGP("snooze");
  display_str("snoozing");
  displaymode = NONE;  // block time display
  delayms(1500);
  displaymode = SHOW_TIME;
}

// we reset the watchdog timer 
void kickthedog(void) {
  wdt_reset();
}

// called @ (F_CPU/256) = ~30khz (31.25 khz)
//SIGNAL (SIG_OVERFLOW0) {  // 168 only
SIGNAL (TIMER0_OVF_vect) {  // 328p & 168
  // allow other interrupts to go off while we're doing display updates
  sei();

  // wake up the dog
  kickthedog();

  // divide down to 100Hz * digits (900 hz)
  muxdiv++;
  if (muxdiv < MUX_DIVIDER)  // MUX_DIVIDER = (300/DISPLAYSIZE) = 33
    return;
  muxdiv = 0;
  // now at 100Hz * digits (900hz - wbp)

  // ok its not really 1ms but its like within 10% :)
  milliseconds++;  // 0.0011111...

  // Cycle through each digit in the display
  // Set the current display's segments
  setdisplay(currdigit);
  // and go to the next
  currdigit++;

#ifdef FEATURE_LDBB
	// currdigit is allowed to go past the end of display bytes.
	// setdisplay will limit the max index value
	// this has the effect of displaying the last digit multiple times
  if (currdigit > lastdigit+ldbb)  // hack to display last digit twice - wbp
    currdigit = 0;  // start over with first digit
#else
  if (currdigit > lastdigit)  // hack to display last digit twice - wbp
    currdigit = 0;  // start over with first digit
#endif

#ifdef FEATURE_SECSMODE
	if (secsmode == 4 && displaymode == SHOW_TIME)  {
		secsdiv++;
		if (secsdiv > SECS_DIVIDER) {
			secsdiv = 0;
			uint16_t ss = milliseconds/167%6*2;  // 6 steps, 1 every 1/6 second
			display[7] = pgm_read_byte(dialsegs4_p + ss + 1);
			display[8] = pgm_read_byte(dialsegs4_p + ss);
		}
	}
#endif

  // check if we should have the alarm on
  if (alarming && !snoozetimer) {
    alarmdiv++;
    if (alarmdiv > ALARM_DIVIDER) { 	// This part only gets reached at 10Hz (wbp)
			alarmdiv = 0;

			alarm_timer++;  // count how long alarm has been beeping
//			if (alarm_timer>1800) {  // kill alarm after 3 minutes...  3*60*10=1800 (testing)
			if (alarm_timer>18000) {  // kill alarm after 30 minutes...  30*60*10=18000 (no body home?)
				alarming=0;  // turn alarm off
				snoozetimer = 0;  // kill snooze timer
				TCCR1B &= ~_BV(CS11); // turn buzzer off
				PORTB |= _BV(SPK1) | _BV(SPK2);
			}

			// start out by beeping once every 20 seconds
			// after each cycle, subtract 2 seconds & add 1 beep
			alarm_count++;
			if (alarm_count > alarm_cycle) {  // once every alarm_cycle
				beep_count = alarm_count = 0;  // restart cycle 
				if (alarm_cycle>20)  // if more than 2 seconds
					alarm_cycle = alarm_cycle - 20;  // subtract 2 seconds
				if (beep_cycle<20)
					beep_cycle += 2;  // add another beep (each beep is two cycles)
			}
			beep_count++;
			if (beep_count <= beep_cycle) {  // how many beeps this cycle?
				// set the PWM output to match the desired frequency
				ICR1 = (F_CPU/8)/(428 + (beep_count*12));  // slowly increasing tone ???
				// we want 50% duty cycle square wave
				OCR1A = OCR1B = ICR1/2;
				if (beep_count%2) {  // odd or even? 
					TCCR1B |= _BV(CS11); // odd, beep on
				} else {
					TCCR1B &= ~_BV(CS11);  // even, beep off
				}
			}
		}  // if (alarmdiv > ALARM_DIVIDER)
		
	}  // if (alarming & !snoozetimer)
		
}
  

// We use the pin change interrupts to detect when buttons are pressed

// These store the current button states for all 3 buttons. We can 
// then query whether the buttons are pressed and released or pressed
// This allows for 'high speed incrementing' when setting the time
volatile uint8_t last_buttonstate = 0, just_pressed = 0, pressed = 0;
volatile uint8_t buttonholdcounter = 0;

// This interrupt detects switches 1 and 3
//SIGNAL(SIG_PIN_CHANGE2) {  // 168 only
SIGNAL(PCINT2_vect) {  // 328p & 168
  // allow interrupts while we're doing this
  PCMSK2 = 0;
  sei();
  // kick the dog
  kickthedog();

  if (! (PIND & _BV(BUTTON1))) {
    // button1 is pressed
    if (! (last_buttonstate & 0x1)) { // was not pressed before
      delayms(10);                    // debounce
      if (PIND & _BV(BUTTON1))        // filter out bounces
      {
        PCMSK2 = _BV(PCINT21) | _BV(PCINT20);
         return;
      }
      tick();                         // make a noise
      // check if we will snag this button press for snoozing
      if (alarming) {
				// turn on snooze
				setsnooze();
				PCMSK2 = _BV(PCINT21) | _BV(PCINT20);
				return;
      }
      last_buttonstate |= 0x1;
      just_pressed |= 0x1;
//      DEBUGP("b1");
    }
  } else {
    last_buttonstate &= ~0x1;
  }

  if (! (PIND & _BV(BUTTON3))) {
    // button3 is pressed
    if (! (last_buttonstate & 0x4)) { // was not pressed before
      delayms(10);                    // debounce
      if (PIND & _BV(BUTTON3))        // filter out bounces
      {
        PCMSK2 = _BV(PCINT21) | _BV(PCINT20);
				return;
      }
			tick();  // wbp
      buttonholdcounter = 2;          // see if we're press-and-holding
      while (buttonholdcounter) {
				if (PIND & _BV(BUTTON3)) {        // released
//					tick();                         // make a noise (wrong place - wbp)
					last_buttonstate &= ~0x4;
					// check if we will snag this button press for snoozing
					if (alarming) {
						// turn on snooze
						setsnooze();
						PCMSK2 = _BV(PCINT21) | _BV(PCINT20);
						return;
					}
//				DEBUGP("b3");
				just_pressed |= 0x4;
				PCMSK2 = _BV(PCINT21) | _BV(PCINT20);
				return;
				}
      }
      last_buttonstate |= 0x4;
      pressed |= 0x4;                 // held down
    }
  } else {
    pressed = 0;                      // button released
    last_buttonstate &= ~0x4;
  }
  PCMSK2 = _BV(PCINT21) | _BV(PCINT20);
}

// Just button #2
//SIGNAL(SIG_PIN_CHANGE0) {  // 168 only
SIGNAL(PCINT0_vect) {  // 328p & 168
  PCMSK0 = 0;
  sei();
  if (! (PINB & _BV(BUTTON2))) {
    // button2 is pressed
    if (! (last_buttonstate & 0x2)) { // was not pressed before
      delayms(10);                    // debounce
      if (PINB & _BV(BUTTON2))        // filter out bounces
      {
        PCMSK0 = _BV(PCINT0);
				return;
      }
      tick();                         // make a noise
      // check if we will snag this button press for snoozing
      if (alarming) {
				setsnooze(); 	// turn on snooze
				PCMSK0 = _BV(PCINT0);
				return;
      }
      last_buttonstate |= 0x2;
      just_pressed |= 0x2;
//      DEBUGP("b2");
    }
  } else {
    last_buttonstate &= ~0x2;
  }
  PCMSK0 = _BV(PCINT0);
}
// This variable keeps track of whether we have not pressed any
// buttons in a few seconds, and turns off the menu display
volatile uint8_t timeoutcounter = 0;

#ifdef FEATURE_DRIFTCORR
/*
 * This goes off once a second, driven by the external 32.768kHz
 * crystal.  It leaves interrupts disabled so it can never itself be
 * interrupted.
 */
SIGNAL (TIMER2_COMPA_vect) {
#else
// this goes off once a second
SIGNAL (TIMER2_OVF_vect) {
#endif

  CLKPR = _BV(CLKPCE);  //MEME
  CLKPR = 0;

  time_s++;    // one second has gone by
  fix_time();  // fix up time values that overflow

#ifdef FEATURE_DRIFTCORR
  /* Apply drift correction on the first second of each hour */
//	OCR2A = DRIFT_BASELINE + drift_corr;  // always apply it (testing) - !!!
  if (time_m == 0) {
    if (time_s == 0)
			OCR2A = DRIFT_BASELINE + drift_corr;
    else if (time_s == 1)
			OCR2A = DRIFT_BASELINE;
    if (time_s <= 1) {
			/* wait for update to take effect (should only be a couple of CPU cycles) */
			while (ASSR & _BV(OCR2AUB))
				;
    }
  }
#endif  

  // If we're in low power mode we should get out now since the display is off
  if (sleepmode)
    return;
	if (gpsTimeout <= gpsTimeout2)
		gpsTimeout ++;  // 1 second has gone by
	
  if (displaymode == SHOW_TIME) {
    if (!timeknown && (time_s % 2)) {  // if time unknown, blink the display
      display_str("        ");
    } else if ( gpsEnabled && (gpsTimeout>gpsTimeout2) && (secsmode!=9) && (time_s % 2) ) {  // if no data from gps
      display_str(" no gps ");
    } else {
      display_time(time_h, time_m, time_s);
    }
    if (alarm_on)
      display[0] |= 0x2;
    else 
      display[0] &= ~0x2;
  }

	if (timeknown)  // 18nov11/wbp
		check_alarm(time_h, time_m, time_s);

#ifdef FEATURE_AUTODIM
  dimmer_update();
#endif

  if (timeoutcounter)
    timeoutcounter--;
  if (buttonholdcounter)
    buttonholdcounter--;
  if (snoozetimer) {
    snoozetimer--;
    if (snoozetimer % 2) 
      display[0] |= 0x2;
    else
      display[0] &= ~0x2;
		if (snoozetimer == 0)  // snooze timer expired?
			start_alarm();  // start the alarm beeping again
  }
}

//SIGNAL(SIG_INTERRUPT0) {  // 168 only
SIGNAL(INT0_vect) {  // 328p & 168
  EIMSK = 0;  //Disable this interrupt while we are processing it.
  uart_putchar('i');
  uint8_t x = ALARM_PIN & _BV(ALARM);
  sei();
  delayms(10); // wait for debouncing
  if (x != (ALARM_PIN & _BV(ALARM)))
  {
    EIMSK = _BV(INT0);
    return;
  }
  setalarmstate();
  EIMSK = _BV(INT0);  //And reenable it before exiting.
}


//SIGNAL(SIG_COMPARATOR) {  // 168 only
SIGNAL(ANALOG_COMP_vect) {  // 328p & 168
  //DEBUGP("COMP");
  if (ACSR & _BV(ACO)) {  // If AC power has been removed
    //DEBUGP("LOW");
    if (!sleepmode) {  // about to go into sleep mode (why is this not in gotosleep???)
      VFDSWITCH_PORT |= _BV(VFDSWITCH); // turn off display
      VFDCLK_PORT &= ~_BV(VFDCLK) & ~_BV(VFDDATA); // no power to vfdchip
      BOOST_PORT &= ~_BV(BOOST); // pull boost fet low
      SPCR  &= ~_BV(SPE); // turn off spi
      if (timeknown) {  // if valid values for time, save in ee  (17nov11/wbp)
				eeprom_update_byte((uint8_t *)EE_HOUR, time_h);
				eeprom_update_byte((uint8_t *)EE_MIN, time_m);
				eeprom_update_byte((uint8_t *)EE_SEC, time_s);
				beep(2000,1);
			}
			if (dateknown) {  // save date if known
				eeprom_update_byte((uint8_t *)EE_YEAR, date_y);    
				eeprom_update_byte((uint8_t *)EE_MONTH, date_m);    
				eeprom_update_byte((uint8_t *)EE_DAY, date_d);    
				beep(2000,2);
      }
//      DEBUGP("z");
      TCCR0B = 0; // no boost
      volume = 0; // low power buzzer
      PCICR = 0;  // ignore buttons
#ifdef FEATURE_AUTODIM
      DIMMER_POWER_PORT &= ~_BV(DIMMER_POWER_PIN); // no power to photoresistor
#endif
      app_start();
    }
  } else {  // AC power available
    //DEBUGP("HIGH");
    if (sleepmode) {  // were we sleeping?
//      if (restored) {  // why bother to write values back to ee prom that were just read?????
//				eeprom_update_byte((uint8_t *)EE_MIN, time_m);
//				eeprom_update_byte((uint8_t *)EE_SEC, time_s);
//      }
//      DEBUGP("WAKERESET"); 
      app_start();
    }
  }
}

/*********************** Main app **********/
void initeeprom(void) {
  if(eeprom_read_byte((uint8_t *)EE_INIT)!=1)
  {
    eeprom_write_byte((uint8_t*)EE_INIT, 1);  //Initialize one time.
    eeprom_write_byte((uint8_t*)EE_YEAR, 13);
    eeprom_write_byte((uint8_t*)EE_MONTH, 6);
    eeprom_write_byte((uint8_t*)EE_DAY, 15);   //Jan 1, 2000
    eeprom_write_byte((uint8_t*)EE_HOUR, 0);
    eeprom_write_byte((uint8_t*)EE_MIN, 0);
    eeprom_write_byte((uint8_t*)EE_SEC, 0);   //00:00:00 (24Hour), 12:00:00 AM (12Hour)
    eeprom_write_byte((uint8_t*)EE_ALARM_HOUR, 10);
    eeprom_write_byte((uint8_t*)EE_ALARM_MIN, 0);   //Alarm 10:00:00/10:00:00AM
    eeprom_write_byte((uint8_t*)EE_BRIGHT, 50);     //Brightness Level = 50
    eeprom_write_byte((uint8_t*)EE_VOLUME, 0);      //Volume Low
    eeprom_write_byte((uint8_t*)EE_REGION, REGION_US);  //12 Hour mode
    eeprom_write_byte((uint8_t*)EE_SNOOZE, 10);     //10 Minute Snooze. (If compiled in.)
    eeprom_write_byte((uint8_t*)EE_ZONE_HOUR, 12-8);     //Zone Hour (GPS) +12
    eeprom_write_byte((uint8_t*)EE_ZONE_MIN, 0);     //Zone Minute (GPS)
    eeprom_write_byte((uint8_t*)EE_DSTMODE, 0);      //No Daylight Saving Time
    eeprom_write_byte((uint8_t*)EE_GPSENABLE, 0);    //GPS disabled
    eeprom_write_byte((uint8_t*)EE_AUTODIMLO, BRITE_MIN);     //Brightness Level Low = 30
    eeprom_write_byte((uint8_t*)EE_AUTODIMHI, BRITE_MAX);     //Brightness Level High = 90
    eeprom_write_byte((uint8_t*)EE_AUTODIM, 0);   //AUTODIM disabled
//uint8_t dst_rules[9]={3,1,2,2,11,1,1,2,1};  
		uint8_t i;
		for (i = 0; i < 9; i++) {
			eeprom_write_byte((uint8_t*)EE_DST_RULE0+i, dst_rules[i]);   //DST RULE 
		}
		eeprom_write_byte((uint8_t *)EE_DSTOFFSET, 0);  // DST OFFSET
    eeprom_write_byte((uint8_t*)EE_SECSMODE, 0);   // Seconds Mode
    eeprom_write_byte((uint8_t*)EE_DRIFTCORR, 0);   // Drift Correction
    eeprom_write_byte((uint8_t*)EE_LDBB, 0);   //Last Digit Brightness Boost
    
    eeprom_write_byte((uint8_t*)EE_TEST, 42);  //In memory of Douglas Adams

    beep(3000,2);                                   //And acknowledge EEPROM written.
  }
}

uint32_t t;

void gotosleep(void) {
  // battery
  //if (sleepmode) //already asleep?
  //  return;
  //DEBUGP("sleeptime");
  
  sleepmode = 1;
  VFDSWITCH_PORT |= _BV(VFDSWITCH); // turn off display
  SPCR  &= ~_BV(SPE); // turn off spi
  VFDCLK_PORT &= ~_BV(VFDCLK) & ~_BV(VFDDATA); // no power to vfdchip
  BOOST_PORT &= ~_BV(BOOST); // pull boost fet low
  TCCR0B = 0; // no boost
  volume = 0; // low power buzzer
  PCICR = 0;  // ignore buttons
#ifdef FEATURE_AUTODIM
  DIMMER_POWER_PORT &= ~_BV(DIMMER_POWER_PIN); // no power to photoresistor
#endif

  // sleep time!
  //beep(3520, 1);
  //beep(1760, 1);
  //beep(880, 1);
  // turn beeper off
  PORTB &= ~_BV(SPK1) & ~_BV(SPK2); 
  
  // turn off pullups
  PORTD &= ~_BV(BUTTON1) & ~_BV(BUTTON3);
  PORTB &= ~_BV(BUTTON2);
  DDRD &= ~_BV(BUTTON1) & ~_BV(BUTTON3);
  DDRB &= ~_BV(BUTTON2);
  ALARM_PORT &= ~_BV(ALARM);
  ALARM_DDR &= ~_BV(ALARM);
  

  // reduce the clock speed
  CLKPR = _BV(CLKPCE);
  CLKPR = _BV(CLKPS3);
  
  //  PPR |= _BV(PRUSART0) | _BV(PRADC) | _BV(PRSPI) | _BV(PRTIM1) | _BV(PRTIM0) | _BV(PRTWI);
  PORTC |= _BV(4);  // sleep signal
  SMCR |= _BV(SM1) | _BV(SM0) | _BV(SE); // sleep mode
  asm("sleep"); 
  CLKPR = _BV(CLKPCE);
  CLKPR = 0;
  PORTC &= ~_BV(4);
}


void initbuttons(void) {
    DDRB =  _BV(VFDCLK) | _BV(VFDDATA) | _BV(SPK1) | _BV(SPK2);
    DDRD = _BV(BOOST) | _BV(VFDSWITCH);
    DDRC = _BV(VFDLOAD) | _BV(VFDBLANK);
    PORTD = _BV(BUTTON1) | _BV(BUTTON3) | _BV(ALARM);
    PORTB = _BV(BUTTON2);

    PCICR = _BV(PCIE0) | _BV(PCIE2);
    PCMSK0 = _BV(PCINT0);
    PCMSK2 = _BV(PCINT21) | _BV(PCINT20);    
}

//
// --------------------------------------- MAIN LOOP -------------------------------------
//
int main(void) {
  uint8_t mcustate;
#ifdef FEATURE_TESTMODE
  uint8_t test=0;
  
  if (! (PIND & _BV(BUTTON1))) {
    if(!(PIND & _BV(BUTTON3))) {
      test=1;
    }
  }
#endif
  // turn boost off
  TCCR0B = 0;
  BOOST_DDR |= _BV(BOOST);
  BOOST_PORT &= ~_BV(BOOST); // pull boost fet low

  // check if we were reset
  mcustate = MCUSR;
  MCUSR = 0;

  uart_putw_hex(mcustate);

  wdt_disable();
  // now turn it back on... 2 second time out
  //WDTCSR |= _BV(WDP0) | _BV(WDP1) | _BV(WDP2);
  //WDTCSR = _BV(WDE);
  wdt_enable(WDTO_2S);
  kickthedog();

  // we lost power at some point so lets alert the user
  // that the time may be wrong (the clock still works)
  dateknown = timeknown = 0;
	
#ifdef FEATURE_WmDST
	dst_update = DST_NO;  // block DST updates until clock init complete
#endif

  // have we read the time & date from eeprom?
  restored = 0;

  // setup uart
//  uart_init(BRRL_192);
  uart_init(BRRL_4800);
	
  //DEBUGP("VFD Clock");
//  DEBUGP("!");

  //DEBUGP("turning on anacomp");
  // set up analog comparator
  ACSR = _BV(ACBG) | _BV(ACIE); // use bandgap, intr. on toggle!
  _delay_ms(1);
  // settle!
  if (ACSR & _BV(ACO)) {  // low power mode ???
    // hmm we should not interrupt here
    ACSR |= _BV(ACI);

    // even in low power mode, we run the clock 
//    DEBUGP("clock init");
    clock_init();  // restores time & date from ee, enables interrupts

  } else {
    // we aren't in low power mode so init stuff

    // init io's
    initbuttons();
    
    VFDSWITCH_PORT &= ~_BV(VFDSWITCH);
    
//    DEBUGP("turning on buttons");
    // set up button interrupts
//    DEBUGP("turning on alarmsw");
    // set off an interrupt if alarm is set or unset
    EICRA = _BV(ISC00);
    EIMSK = _BV(INT0);
  
    displaymode = SHOW_TIME;
//    DEBUGP("vfd init");
    vfd_init();

#ifdef FEATURE_AUTODIM
//    DEBUGP("boost init");
		autodim_lo = eeprom_read_byte((uint8_t *)EE_AUTODIMLO);
		autodim_hi = eeprom_read_byte((uint8_t *)EE_AUTODIMHI);
#endif
    bright_level = eeprom_read_byte((uint8_t *)EE_BRIGHT);
#ifdef FEATURE_AUTODIM
    autodim = eeprom_read_byte((uint8_t *)EE_AUTODIM);
		if (autodim == AUTODIM_ON)
			boost_init(autodim_hi);  // make greeting visible
		else
#endif
			boost_init(bright_level);
    sei();

    //Load and check the timezone information
    intTimeZoneHour = eeprom_read_byte((uint8_t *)EE_ZONE_HOUR) - 12;
    if ( ( 12 < intTimeZoneHour ) || ( -12 > intTimeZoneHour ) )
      intTimeZoneHour = 0;

    intTimeZoneMin = eeprom_read_byte((uint8_t *)EE_ZONE_MIN);
    if ( ( 60 < intTimeZoneMin ) || ( 0 > intTimeZoneMin ) )
      intTimeZoneMin = 0;
    
//    DEBUGP("speaker init");
    speaker_init();
    
//    DEBUGP("eeprom init");  //Reset eeprom to defaults, if completely blank.
    initeeprom();

#ifdef FEATURE_AUTODIM
    dimmer_init();
#endif

// check EE memory
uint8_t b1, b2;
	b1 = eeprom_read_byte((uint8_t *)EE_INIT);
	b2 = eeprom_read_byte((uint8_t *)EE_TEST);
	if ((b1!=1) || (b2!=42)) {
		beep_ms(440,2,500);  // indicate error with 2 long beeps
	}

    region = eeprom_read_byte((uint8_t *)EE_REGION); 
#ifdef FEATURE_WmDST
    dst_mode = eeprom_read_byte((uint8_t *)EE_DSTMODE);
		dst_offset = eeprom_read_byte((uint8_t *)EE_DSTOFFSET);  // get last known DST Offset
		dst_rules[0] = eeprom_read_byte((uint8_t *)EE_DST_RULE0);  // read DST rules from EE prom
		dst_rules[1] = eeprom_read_byte((uint8_t *)EE_DST_RULE1);  // read DST rules from EE prom
		dst_rules[2] = eeprom_read_byte((uint8_t *)EE_DST_RULE2);  // read DST rules from EE prom
		dst_rules[3] = eeprom_read_byte((uint8_t *)EE_DST_RULE3);  // read DST rules from EE prom
		dst_rules[4] = eeprom_read_byte((uint8_t *)EE_DST_RULE4);  // read DST rules from EE prom
		dst_rules[5] = eeprom_read_byte((uint8_t *)EE_DST_RULE5);  // read DST rules from EE prom
		dst_rules[6] = eeprom_read_byte((uint8_t *)EE_DST_RULE6);  // read DST rules from EE prom
		dst_rules[7] = eeprom_read_byte((uint8_t *)EE_DST_RULE7);  // read DST rules from EE prom
		dst_rules[8] = eeprom_read_byte((uint8_t *)EE_DST_RULE8);  // read DST rules from EE prom
#endif
#ifdef FEATURE_GPS
    gpsEnabled = eeprom_read_byte((uint8_t *)EE_GPSENABLE);
		if (gpsEnabled == GPS_48)
		  uart_init(BRRL_4800);
		else if (gpsEnabled == GPS_96)
		  uart_init(BRRL_9600);
#endif
#ifdef FEATURE_SECSMODE
    secsmode = eeprom_read_byte((uint8_t *)EE_SECSMODE);
#endif
#ifdef FEATURE_LDBB
    ldbb = eeprom_read_byte((uint8_t *)EE_LDBB);
		if (ldbb > 1)
			ldbb = 0;
#endif			

		displaymode = NONE;  // prevent display update when clock ticks

		display_str("ice tube");  // say hello
		beep_ms(3520,1,100);
		_delay_ms(500);  // wait a bit without enabling interrupts  (wbp)

//    DEBUGP("clock init");
    clock_init();  // restores time & date from ee, enables interrupts
	
    kickthedog();
    beep_ms(3520,1,100);  // (wbp)
		display_str("        ");  // clear screen
		delayms(200);
		displaymode = SHOW_TIME;

#ifdef FEATURE_WmDST
//		setDSToffset();  // set DST offset based on restored values
	dst_update = DST_YES;  // OK to update DST Offset now
#endif
		
//    DEBUGP("alarm init");
    setalarmstate();

#ifdef FEATURE_TESTMODE
    if(test)
    {
      displaymode = TESTMODE;
      testmode(0);
      displaymode = SHOW_TIME;
    }
#endif
  }
//  DEBUGP("done");
//
// --------------------------------------- MENU LOOP -------------------------------------
//
  while (1) {
    //_delay_ms(100);
    kickthedog();
    //uart_putc_hex(ACSR);
    if (ACSR & _BV(ACO)) {  // low power mode?
      // DEBUGP("SLEEPYTIME");
      gotosleep();
      continue;
    }
    //DEBUGP(".");
    if (just_pressed & 0x1) {  // Mode button
      just_pressed = 0;
			if (displaymode != NONE)
				displaymode = (displaymode+1)%N_MODES;  // increment menu, wrap
      switch(displaymode) {
      case (SHOW_TIME):
				break;
      case (SHOW_ABOUT):
				display_str("ice tube");
				show_about();
				break;
      case (SET_ALARM):
				display_str("set alar");
				set_alarm();
				break;
      case (SET_TIME):
				display_str("set time");
				set_time();
				timeknown = 1;
				break;
      case (SET_DATE):
				display_str("set date");
				set_date();
				dateknown = 1;
				break;
      case (SET_ZONE):
        display_str("set zone");
        set_timezone();
				break;
#ifdef FEATURE_GPS
      case (SET_GPSENABLE):
				display_str("set gps ");
				set_gpsenable();
				break;
#endif
#ifdef FEATURE_AUTODIM
      case (SET_AUTODIM):  // auto dim on/off
				display_str("set adim");
				set_autodim();
				break;
#endif
      case (SET_BRIGHTNESS):
				display_str("set brit");
#ifdef FEATURE_AUTODIM
				if (autodim)
					set_autobrightness();
				else
#endif
					set_brightness();
				break;
#ifdef FEATURE_DRIFTCORR
      case (SET_DRIFTCORR):
				display_str("set drft");
				set_driftcorr();
				break;
#endif
			case (SET_REGION):    	
				display_str("set regn");
				set_region();
				break;
#ifdef FEATURE_SECSMODE
      case (SET_SECSMODE):
				display_str("set secs");
				set_secsmode();
				break;
#endif
#ifdef FEATURE_SETSNOOZE 
      case (SET_SNOOZETIME):
				display_str("set snoz");
				set_snoozetime();
				break;
#endif
      case (SET_VOLUME):
				display_str("set vol ");
				set_volume();
				break;
#ifdef FEATURE_WmDST
      case (SET_DSTMODE):
				display_str("set dst ");
				set_dstmode();
				break;
      case (SET_DSTRULES):
				display_str("set rule");
				set_dstrules();
				break;
#endif
#ifdef FEATURE_LDBB
      case (SET_LDBB):
				display_str("set ldbb");
				set_ldbb();
				break;
#endif				
#ifdef FEATURE_TESTMODE
      case (TESTMODE):
				display_str("testmode");
				set_test();
				break;
#endif
      default:
				break;
      }
    } 
		else if ((just_pressed & 0x2) || (just_pressed & 0x4)) {
      just_pressed = 0;
      displaymode = NONE;  // this locks out button pushes
      display_date(DAY);
      kickthedog();
      delayms(1000);  // pause 1 second (locks out all button pushes)
      kickthedog();
      displaymode = SHOW_TIME;     
    } 
#ifdef FEATURE_GPS
    //Check to see if GPS data is ready:
		if (gpsEnabled > GPS_OFF)
		{
			if ( gpsDataReady() )   // if there is data in the buffer
			getGPSdata();  // get the GPS serial stream and update the clock 
		}
#endif
	}
}

/**************************** SUB-MENUS *****************************/

void set_alarm(void) 
{
  uint8_t mode;
  uint8_t hour, min, sec;
  hour = min = sec = 0;
  mode = SHOW_MENU;
  hour = alarm_h;
  min = alarm_m;
  sec = 0;
  timeoutcounter = INACTIVITYTIMEOUT;
  while (1) {
    if (just_pressed & 0x1) { // mode change
      return;
    }
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;  
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
      displaymode = SHOW_TIME;     
      alarm_h = hour;
      alarm_m = min;
      eeprom_update_byte((uint8_t *)EE_ALARM_HOUR, alarm_h);    
      eeprom_update_byte((uint8_t *)EE_ALARM_MIN, alarm_m);    
      return;
    }
    if (just_pressed & 0x2) {
      just_pressed = 0;
      if (mode == SHOW_MENU) {
				// ok now its selected
				mode = SET_HOUR;
				display_alarm(hour, min);
				display[1] |= 0x1;
				display[2] |= 0x1;	
      } else if (mode == SET_HOUR) {
				mode = SET_MIN;
				display_alarm(hour, min);
				display[4] |= 0x1;
				display[5] |= 0x1;
      } else {
				// done!
				alarm_h = hour;
				alarm_m = min;
				eeprom_update_byte((uint8_t *)EE_ALARM_HOUR, alarm_h);    
				eeprom_update_byte((uint8_t *)EE_ALARM_MIN, alarm_m);    
				displaymode = SHOW_TIME;
				return;
      }
    }
    if ((just_pressed & 0x4) || (pressed & 0x4)) {
      just_pressed = 0;
      if (mode == SET_HOUR) {
				hour = (hour+1) % 24;
				display_alarm(hour, min);
				display[1] |= 0x1;
				display[2] |= 0x1;
      }
      if (mode == SET_MIN) {
				min = (min+1) % 60;
				display_alarm(hour, min);
				display[4] |= 0x1;
				display[5] |= 0x1;
      }
      if (pressed & 0x4) {
				tick();  // make a noise (wbp)
				delayms(75);
			}
    }
  }
}

void show_about(void)
{
  timeoutcounter = INACTIVITYTIMEOUT;
  while (1) {
    if (just_pressed & 0x1) { // mode change
      return;
    }
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (just_pressed & 0x2) {
      just_pressed = 0;
			display_str(version);
		}
	}
}
	
void set_time(void) 
{
  uint8_t mode;
  uint8_t hour, min, sec;
    
  hour = time_h;
  min = time_m;
  sec = time_s;
  mode = SHOW_MENU;

  timeoutcounter = INACTIVITYTIMEOUT;
  
  while (1) {
    if (just_pressed & 0x1) { // mode change
      return;
    }
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (just_pressed & 0x2) {  // select
      just_pressed = 0;
      if (mode == SHOW_MENU) {
				hour = time_h;
				min = time_m;
				sec = time_s;
				// ok now its selected
				mode = SET_HOUR;
				display_time(hour, min, sec);
				display[1] |= 0x1;
				display[2] |= 0x1;	
      } else if (mode == SET_HOUR) {
				mode = SET_MIN;
				display_time(hour, min, sec);
				display[4] |= 0x1;
				display[5] |= 0x1;
      } else if (mode == SET_MIN) {
				mode = SET_SEC;
				display_time(hour, min, sec);
				display[7] |= 0x1;
				display[8] |= 0x1;
      } else {
				// done!
				time_h = hour;
				time_m = min;
				time_s = sec;
				displaymode = SHOW_TIME;
#ifdef FEATURE_WmDST
		    if (dst_mode == DST_AUTO) { // Is DST set to Auto? 16jun13/wbp
					dst_update = DST_YES;  // Reset DST Update flag
					setDSToffset();  // Setup Auto DST per current rules
				}
//				dst_update = DST_YES;  // Reset DST Update flag in case time to adjust is soon
//				time_h = hour;  // set hour back to what was displayed
#endif
				eeprom_update_byte((uint8_t *)EE_HOUR, time_h);    
				eeprom_update_byte((uint8_t *)EE_MIN, time_m);
				eeprom_update_byte((uint8_t *)EE_SEC, time_s);
				return;
      }
    }
    if ((just_pressed & 0x4) || (pressed & 0x4)) {  // adjust
      just_pressed = 0;
      if (mode == SET_HOUR) {
				hour = (hour+1) % 24;
				display_time(hour, min, sec);
				display[1] |= 0x1;
				display[2] |= 0x1;
				time_h = hour;
//				eeprom_update_byte((uint8_t *)EE_HOUR, time_h);    
			}
      if (mode == SET_MIN) {
				min = (min+1) % 60;
				display_time(hour, min, sec);
				display[4] |= 0x1;
				display[5] |= 0x1;
//				eeprom_update_byte((uint8_t *)EE_MIN, time_m);
				time_m = min;
			}
      if ((mode == SET_SEC) ) {
				sec = (sec+1) % 60;
				display_time(hour, min, sec);
				display[7] |= 0x1;
				display[8] |= 0x1;
				time_s = sec;
      }
      
      if (pressed & 0x4) {
				tick();  // make a noise (wbp)
				delayms(75);
			}
		}
  }
}

void set_date(void) {
  uint8_t mode = SHOW_MENU;
  timeoutcounter = INACTIVITYTIMEOUT;
  while (1) {
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (just_pressed & 0x1) { // mode change
      return;
    }
    if (just_pressed & 0x2) {
      just_pressed = 0;
      if (mode == SHOW_MENU) {
	// start!
				if (region == REGION_US) {
				mode = SET_MONTH;
				}
				else {
//					DEBUGP("Set day");
					mode = SET_DAY;
				}
				display_date(DATE);
				display[1] |= 0x1;
				display[2] |= 0x1;
      } else if (((mode == SET_MONTH) && (region == REGION_US)) || ((mode == SET_DAY) && (region == REGION_EU))) {
				if (region == REGION_US)
					mode = SET_DAY;
				else
					mode = SET_MONTH;
				display_date(DATE);
				display[4] |= 0x1;
				display[5] |= 0x1;
			} else if (((mode == SET_DAY) && (region == REGION_US)) || ((mode == SET_MONTH) && (region == REGION_EU))) {
				mode = SET_YEAR;
				display_date(DATE);
				display[7] |= 0x1;
				display[8] |= 0x1;
			} else {
				displaymode = NONE;
				display_date(DATE);
				delayms(1500);
				displaymode = SHOW_TIME;
#ifdef FEATURE_WmDST
				if (dst_mode == DST_AUTO) {
					dst_update = DST_YES;  // Reset DST Update flag
					setDSToffset();  // Date changed, set DST offset per current rules
				}
#endif
				eeprom_update_byte((uint8_t *)EE_YEAR, date_y);    
				eeprom_update_byte((uint8_t *)EE_MONTH, date_m);    
				eeprom_update_byte((uint8_t *)EE_DAY, date_d);    
				return;
			}
    }
    if ((just_pressed & 0x4) || (pressed & 0x4)) {
      just_pressed = 0;
      if (mode == SET_MONTH) {
				date_m++;
				if (date_m >= 13)
					date_m = 1;
				display_date(DATE);
				if (region == REGION_US) {
					display[1] |= 0x1;
					display[2] |= 0x1;
				} else {
					display[4] |= 0x1;
					display[5] |= 0x1;
				}
//				eeprom_update_byte((uint8_t *)EE_MONTH, date_m);    
      }
      if (mode == SET_DAY) {
				date_d++;
				if (date_d > 31)
					date_d = 1;
				display_date(DATE);
				if (region == REGION_EU) {
					display[1] |= 0x1;
					display[2] |= 0x1;
				} else {
					display[4] |= 0x1;
					display[5] |= 0x1;
				}
//				eeprom_update_byte((uint8_t *)EE_DAY, date_d);    
      }
      if (mode == SET_YEAR) {
				date_y++;
				date_y %= 100;
				display_date(DATE);
				display[7] |= 0x1;
				display[8] |= 0x1;
//				eeprom_update_byte((uint8_t *)EE_YEAR, date_y);    
      }

      if (pressed & 0x4) {
				tick();  // make a noise  (wbp)
				delayms(60);
			}
    }
  }
}

//Function to set the time zone
void set_timezone(void) {
  int8_t hour = intTimeZoneHour;
  uint8_t min = intTimeZoneMin;
  uint8_t mode = SHOW_MENU;
  timeoutcounter = INACTIVITYTIMEOUT;
  while (1) {
    if (just_pressed & 0x1) { // mode change
      return;
    }
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;  
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (just_pressed & 0x2) {
      just_pressed = 0;
      if (mode == SHOW_MENU) {
				// ok now its selected
				mode = SET_HOUR;
				display_timezone(hour, min);
				display[1] |= 0x1;
				display[2] |= 0x1;	
      } else if (mode == SET_HOUR) {
				mode = SET_MIN;
				display_timezone(hour, min);
				display[4] |= 0x1;
				display[5] |= 0x1;
      } else {
				// done!
				displaymode = SHOW_TIME;
				return;
      }
    }
    if ((just_pressed & 0x4) || (pressed & 0x4)) {
      just_pressed = 0;
      if (mode == SET_HOUR) {
				hour = ( ( hour + 1 + 12 ) % 25 ) - 12;
				display_timezone(hour, min);
				display[1] |= 0x1;
				display[2] |= 0x1;
        intTimeZoneHour = hour;
				eeprom_update_byte((uint8_t *)EE_ZONE_HOUR, hour+12);
//Debugging:
//				uart_puts("\n\rTimezone offset hour:\t");
//				uart_putw_dec(hour);
      }
      if (mode == SET_MIN) {
				min = ( min + 1 ) % 60;
				display_timezone(hour, min);
				display[4] |= 0x1;
				display[5] |= 0x1;
        intTimeZoneMin = min;
				eeprom_update_byte((uint8_t *)EE_ZONE_MIN, min);
      }
      if (pressed & 0x4) {
				tick();  // make a noise (wbp)
				delayms(75);
			}
    }
  }
}

#ifdef FEATURE_LDBB
void show_ldbb(void) {
	if(ldbb > 0)
		display_str("on  8888");
	else
		display_str("off 8888");
}
void set_ldbb(void) {
  uint8_t mode = SHOW_MENU;
  timeoutcounter = INACTIVITYTIMEOUT;
  while (1) {
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (just_pressed & 0x1) { // mode change
      return;
    }
    if (just_pressed & 0x2) { // select
      just_pressed = 0;
      if (mode == SHOW_MENU) {
				// start!
				mode = SET_LDBB;
				show_ldbb();
				}
      else {	
				eeprom_update_byte((uint8_t *)EE_LDBB, ldbb);
				displaymode = SHOW_TIME;
				return;
			}
		}
    if (just_pressed & 0x4) {
      just_pressed = 0;
      if (mode == SET_LDBB) {
				if (ldbb > 0)
					ldbb = 0;
				else
					ldbb = 1;
				show_ldbb();
				eeprom_update_byte((uint8_t *)EE_LDBB, ldbb);
      }
    }
  }
}
#endif

#ifdef FEATURE_GPS
void show_gpsenabled(void) {
	if (gpsEnabled == GPS_48)
		display_str("gps on48");
	else if (gpsEnabled == GPS_96)
		display_str("gps on96");
	else
		display_str("gps off ");
}
void set_gpsenable(void) {
  uint8_t mode = SHOW_MENU;  // initial value
  timeoutcounter = INACTIVITYTIMEOUT;
  while (1) {
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (just_pressed & 0x1) { // mode change
      return;
    }
    if (just_pressed & 0x2) { // select
      just_pressed = 0;
			mode = (mode+1)%6;  // sequence thru sub menu items
			switch (mode) {
				case SHOW_MENU:
					display_str("set gps ");
					break;
				case SET_GPS:
					show_gpsenabled();
					break;
				case SHOW_GPSLAT1:
					display_str("gps lat ");
					break;
				case SHOW_GPSLAT2:
					show_gpslat();
					break;
				case SHOW_GPSLONG1:
					display_str("gps long");
					break;
				case SHOW_GPSLONG2:
					show_gpslng();
					break;
			}
		}
    if (just_pressed & 0x4) { // increment
      just_pressed = 0;
      if (mode == SET_GPS) {
				if (gpsEnabled == GPS_OFF) {
					gpsEnabled = GPS_48;
					uart_init(BRRL_4800);
					}
				else if (gpsEnabled == GPS_48) {
					gpsEnabled = GPS_96;
					uart_init(BRRL_9600);
					}
				else
					gpsEnabled = GPS_OFF;
				show_gpsenabled();
				eeprom_update_byte((uint8_t *)EE_GPSENABLE, gpsEnabled);
				if (dst_mode == DST_AUTO) {
					dst_update = DST_YES;  // Reset DST Update flag
					setDSToffset();  // Allow DST update
				}
      }
    }
  }
}

void show_gpslat() {
	uint8_t i;
	display[1] = 0;  // blank
  if (gpsLatH[0] == 'N')  // North is positive
    display[2] = 0;  // positive numbers, implicit sign
  else 
    display[2] = 0x2;  // negative numbers, display negative sign
	for (i = 0; i<6; i++) {  // "ddmmff" 
		display[i+3] = get_number(gpsLat[i] - '0');
	}
	display[6] |= 0x1;  // turn on decimal point
}

void show_gpslng() {
	uint8_t i;
  if (gpsLongH[0] == 'E')  // East is positive
    display[1] = 0;  // positive numbers, implicit sign
  else 
    display[1] = 0x2;  // negative numbers, display negative sign
	for (i = 0; i<7; i++) {  // "dddmmff"  
		display[i+2] = get_number(gpsLong[i] - '0');
	}
	display[6] |= 0x1;  // decimal point
}
#endif

void set_brightness(void) {
  uint8_t mode = SHOW_MENU;
  timeoutcounter = INACTIVITYTIMEOUT;
  while (1) {
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
      displaymode = SHOW_TIME;     
      eeprom_update_byte((uint8_t *)EE_BRIGHT, bright_level);
      return;
    }
    if (just_pressed & 0x1) { // mode change
      return;
    }
    if (just_pressed & 0x2) {
      just_pressed = 0;
      if (mode == SHOW_MENU) {
				// start!
				mode = SET_BRITE;
				// display brightness
				display_str("brite ");
				display_brt(bright_level);
      } else {	
				displaymode = SHOW_TIME;
				eeprom_update_byte((uint8_t *)EE_BRIGHT, bright_level);
				return;
      }
    }
    if ((just_pressed & 0x4) || (pressed & 0x4)) {
      just_pressed = 0;
      if (mode == SET_BRITE) {
        // Increment brightness level, wrap around at max
				bright_level += BRITE_INCREMENT;
				if (bright_level > BRITE_MAX) {
					bright_level = BRITE_MIN;
				}
				display_brt(bright_level);
      }
      if (pressed & 0x4) {
				tick();  // make a noise (wbp)
				delayms(75);
			}
    }
  }
}

void display_brt(uint8_t brt) {
//  display[7] = get_number((brt / 10)) | 0x1;
//  display[8] = get_number((brt % 10)) | 0x1;
	display_num(7, brt, 0x1);
  set_vfd_brightness(brt);
}

#ifdef FEATURE_AUTODIM
void show_autodim(void) {
	if(autodim == AUTODIM_ON)
		display_str("adim on ");
	else
		display_str("adim off");
}
void set_autodim(void) {
  uint8_t mode = SHOW_MENU;
  timeoutcounter = INACTIVITYTIMEOUT;
  while (1) {
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (just_pressed & 0x1) { // mode change
      return;
    }
    if (just_pressed & 0x2) { // select
      just_pressed = 0;
      if (mode == SHOW_MENU) {
				// start!
				mode = SET_AUTODIM;
				show_autodim();
			} else if (mode == SET_AUTODIM) {
				mode = SHOW_ADCLEVEL;
				display_str("adc ");
				display_value(1, 4, dimmer_adc);
				display_value(6, 3, dimmer_lvl);
			} else if (mode == SHOW_ADCLEVEL) {
				dimmer_update();  // start ADC 
				display_value(1, 4, dimmer_adc);
				display_value(6, 3, dimmer_lvl);
//				display_value(1, 3, gpsCheck1);
//				display_value(4, 5, gpsCheck2);
			} else {	
				eeprom_update_byte((uint8_t *)EE_AUTODIM, autodim);
				displaymode = SHOW_TIME;
				return;
			}
		}
    if (just_pressed & 0x4) {
      just_pressed = 0;
      if (mode == SET_AUTODIM) {
				if (autodim == AUTODIM_ON)
					autodim = AUTODIM_OFF;
				else
					autodim = AUTODIM_ON;
				show_autodim();
				eeprom_update_byte((uint8_t *)EE_AUTODIM, autodim);
      }
    }
  }
}

void display_value(unsigned char p, unsigned char n, uint16_t val) {
	uint16_t v = val;
	int8_t i;  // must be signed!
	for (i = (n-1); i >= 0; i--) {  
		display[p+i] = get_number(v % 10);
		v = v / 10;
	}
}

void set_autobrightness(void) {  // set brightness levels if autodim on
  uint8_t mode = SHOW_MENU;
  timeoutcounter = INACTIVITYTIMEOUT;
	autodim_lo = eeprom_read_byte((uint8_t *)EE_AUTODIMLO);
	autodim_hi = eeprom_read_byte((uint8_t *)EE_AUTODIMHI);
	autodim = AUTODIM_OFF;  // temporarily suspend auto dimming
  while (1) {
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
      displaymode = SHOW_TIME;     
			autodim = AUTODIM_ON;  // turn AUTODIM back on
      return;
    }
    if (just_pressed & 0x1) { // mode change
			autodim = AUTODIM_ON;  // turn AUTODIM back on
      return;
    }
    if (just_pressed & 0x2) {  // select button
      just_pressed = 0;
      if (mode == SHOW_MENU) {
				mode = SET_AUTODIMLO;
				display_str("brtlo ");
				display_brt(autodim_lo);
      } else if (mode == SET_AUTODIMLO) {
				mode = SET_AUTODIMHI;
				display_str("brthi ");
				display_brt(autodim_hi);
			}
			else {	
				displaymode = SHOW_TIME;
				eeprom_update_byte((uint8_t *)EE_AUTODIMLO, autodim_lo);
				eeprom_update_byte((uint8_t *)EE_AUTODIMHI, autodim_hi);
				autodim = AUTODIM_ON;  // turn AUTODIM back on
				return;
      }
    }
    if ((just_pressed & 0x4) || (pressed & 0x4)) {  // increment
      just_pressed = 0;
      if (mode == SET_AUTODIMLO) {
//				autodim_lo += BRITE_INCREMENT;
//				if (autodim_lo > BRITE_MAX) {
				autodim_lo += 2;
				if (autodim_lo > 50) {
					autodim_lo = AUTO_BRITE_MIN;  // start low level really low
				}
				display_brt(autodim_lo);
//				eeprom_update_byte((uint8_t *)EE_AUTODIMLO, autodim_lo);
      } else if (mode == SET_AUTODIMHI) {
				autodim_hi += BRITE_INCREMENT;
				if (autodim_hi > BRITE_MAX) {
//					autodim_hi = BRITE_MIN;
					autodim_hi = 50;
				}
				display_brt(autodim_hi);
//				eeprom_update_byte((uint8_t *)EE_AUTODIMHI, autodim_hi);
			}
      if (pressed & 0x4) {  // button held down
				tick();  // make a noise (wbp)
				delayms(75);
			}
    }
  }
}
#endif

#ifdef FEATURE_WmDST
void show_dstmode(void) {
	if (dst_mode == DST_AUTO)
		display_str("dst auto");
	else if (dst_mode == DST_ON)
		display_str("dst on  ");
	else
		display_str("dst off ");
}

void set_dstmode(void) {
  uint8_t mode = SHOW_MENU;
  timeoutcounter = INACTIVITYTIMEOUT;
  while (1) {
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (just_pressed & 0x1) {  // mode change
      return;
    }
    if (just_pressed & 0x2) {  // select
			just_pressed = 0;
      if (mode == SHOW_MENU) {
				// start!
				mode = SET_DSTMODE;
				show_dstmode();
			}	else if (mode == SET_DSTMODE) {
				mode = SET_DSTMODE+1;  // actual value doesn't matter as long as it's different
				show_dstoffset();
      } else {	
				displaymode = SHOW_TIME;
				return;
      }
    }
    if (just_pressed & 0x4) {  // adjust
      just_pressed = 0;
      if (mode == SET_DSTMODE) {
				if (dst_mode == DST_ON) {
					dst_mode = DST_AUTO;
					dst_update = DST_YES;  // allow DST Offset to be adjusted
					setDSToffset();  // Set DST offset per current rules
//					dst_update = DST_YES;  // Reset DST Update flag in case time to adjust is soon
				} else if (dst_mode == DST_AUTO) {
					dst_mode = DST_OFF;
					if (dst_offset > 0) {
						dst_offset = 0;  // no offset
						time_h--;  // set clock back 1 hour
						eeprom_update_byte((uint8_t *)EE_DSTOFFSET, dst_offset);
						eeprom_update_byte((uint8_t *)EE_HOUR, time_h);    
					}
				} else {
					dst_mode = DST_ON;
					if (dst_offset == 0)  {
						dst_offset = 1;  // offset
						time_h++;
						eeprom_update_byte((uint8_t *)EE_DSTOFFSET, dst_offset);
						eeprom_update_byte((uint8_t *)EE_HOUR, time_h);    
					}
				}
				show_dstmode();
				eeprom_update_byte((uint8_t *)EE_DSTMODE, dst_mode);
      }
    }
  }
}

void show_dstoffset(void)
{
	display_num(7, dst_offset, 0);
	display_str("offset ");
}

void set_dstrules(void)
{
  uint8_t mode = SHOW_MENU;
	uint8_t iRule = 0;
  timeoutcounter = INACTIVITYTIMEOUT;
//	uint8_t rules_mod[9] = {0,0,0,0,0,0,0,0,0};  // don't really need this now
  while (1) {
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
			save_dstrules();  // save any updated rules
      displaymode = SHOW_TIME;     
      return;
    }
    if (just_pressed & 0x1) { // mode change
			save_dstrules();  // save any updated rules
      return;
    }
    if (just_pressed & 0x2) {  // select
      just_pressed = 0;
      if (mode == SHOW_MENU) {
				mode = SET_DSTRULES;
				display_str("rule0 00");
				display_dstrule(iRule);
      } else if (mode == SET_DSTRULES) {
				iRule ++;
				if (iRule > 8) iRule = 0;  // wrap around - exit with button 0 or timeout
				display_dstrule(iRule);
			}	else {	
				displaymode = SHOW_TIME;
				save_dstrules();  // save any updated rules
				return;
      }
    }
    if ((just_pressed & 0x4) || (pressed & 0x4)) {  // increment
      just_pressed = 0;
      if (mode == SET_DSTRULES) {
				dst_rules[iRule] ++;
				if (dst_rules[iRule] > dst_rules_hi[iRule])
					dst_rules[iRule] = dst_rules_lo[iRule];  // wrap around
//				rules_mod[iRule] = 1;  // at least one rule has been changed
				display_dstrule(iRule);
//				eeprom_update_byte((uint8_t *)EE_DSTRULE0+iRule, dst_rules[iRule]);
      }
      if (pressed & 0x4) {
				tick();  // make a noise (wbp)
				delayms(75);
			}
    }
  }
}

//void save_dstrules(uint8_t mod[9]) {
void save_dstrules() {
//	uint8_t i, ch=0;
	uint8_t i;
	for (i = 0; i<9; i++) {
//		if (mod[i]) { 
//			ch++;
			eeprom_update_byte((uint8_t *)EE_DST_RULE0+i, dst_rules[i]);
//		}
//		if (ch)  {
			dst_update = DST_YES;  // allow DST Offset to be adjusted
	    if (dst_mode == DST_AUTO)  // Is DST set to Auto? 16jun13/wbp
				setDSToffset();  // if rules changed, update DST offset
//		}
	}
}

void display_dstrule(uint8_t i) {
  display[5] = get_number(i) | 0x0;
//  display[7] = get_number((dst_rules[i] / 10)) | 0x1;
//  display[8] = get_number((dst_rules[i] % 10)) | 0x1;
	display_num(7, dst_rules[i], 0x1);
}
#endif

#ifdef FEATURE_TESTMODE
void set_test(void) {
  uint8_t mode = SHOW_MENU;

  timeoutcounter = INACTIVITYTIMEOUT;

  while (1) {
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (just_pressed & 0x1) { // mode change
      return;
    }
    if (just_pressed & 0x2) {
      just_pressed = 0;
      if (mode == SHOW_MENU) {
	// start!
	      testmode(1);
        displaymode = SHOW_TIME;
        return;
      }
    }
    if (just_pressed & 0x4) {
      just_pressed = 0;
    }
  }
}
#endif

void show_volume(void) {
	if (volume) {
		display_str("vol high");
		display[8] |= 0x1;
	} else {
		display_str("vol low ");
	}
	display[5] |= 0x1;
	display[6] |= 0x1;
	display[7] |= 0x1;
}

void set_volume(void) {
  uint8_t mode = SHOW_MENU;
  uint8_t volume;
  timeoutcounter = INACTIVITYTIMEOUT;
  volume = eeprom_read_byte((uint8_t *)EE_VOLUME);
  while (1) {
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (just_pressed & 0x1) { // mode change
      return;
    }
    if (just_pressed & 0x2) {
      just_pressed = 0;
      if (mode == SHOW_MENU) {
				// start!
				mode = SET_VOL;
				show_volume();
      } else {	
				displaymode = SHOW_TIME;
				return;
      }
    }
    if (just_pressed & 0x4) {
      just_pressed = 0;
      if (mode == SET_VOL) {
				volume = !volume;
				eeprom_update_byte((uint8_t *)EE_VOLUME, volume);
				speaker_init();
				beep(4000, 1);
				show_volume();
			}
    }
  }
}

void show_region(void) {
	if (region == REGION_US) {
		display_str("usa-12hr");
	} else {
		display_str("eur-24hr");
	}
}

void set_region(void) {
  uint8_t mode = SHOW_MENU;
  timeoutcounter = INACTIVITYTIMEOUT;
  region = eeprom_read_byte((uint8_t *)EE_REGION);
  while (1) {
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (just_pressed & 0x1) { // mode change
      return;
    }
    if (just_pressed & 0x2) {
      just_pressed = 0;
      if (mode == SHOW_MENU) {
				// start!
				mode = SET_REG;
				// display region
				show_region();
      } else {	
				displaymode = SHOW_TIME;
				return;
      }
    }
    if (just_pressed & 0x4) {
      just_pressed = 0;
      if (mode == SET_REG) {
				region = !region;
				show_region();
				eeprom_update_byte((uint8_t *)EE_REGION, region);
      }
    }
  }
}

void display_snum(unsigned char pos, int8_t d, uint8_t hilite) {  // display signed number
	if (d < 0) {
		display[pos] = 0x02;     // dash
		d = -d;
	}
	else
		display[pos] = 0x00;     // space
	display[pos+1] = get_number((d / 10)) | hilite;
	display[pos+2] = get_number((d % 10)) | hilite;
}

//void display_num(unsigned char pos, int16_t d, uint8_t hilite) {  // display signed number, with optional hilite
void display_num(unsigned char pos, int8_t d, uint8_t hilite) {  // display signed number, with optional hilite
	display[pos+0] = get_number((d / 10)) | hilite;
	display[pos+1] = get_number((d % 10)) | hilite;
}

#ifdef FEATURE_DRIFTCORR
void set_driftcorr(void) {
  uint8_t mode = SHOW_MENU;
  timeoutcounter = INACTIVITYTIMEOUT;
  drift_corr = eeprom_read_byte((uint8_t *)EE_DRIFTCORR);
  while (1) {
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (just_pressed & 0x1) { // mode change
      return;
    }
    if (just_pressed & 0x2) {
      just_pressed = 0;
      if (mode == SHOW_MENU) {
				// start!
				mode = SET_DRIFTCORR;
				// display drift
				display_str("drft  ");
				display_snum(6, drift_corr, 1);
      } else { 
				eeprom_update_byte((uint8_t *)EE_DRIFTCORR, drift_corr);
//				OCR2A = DRIFT_BASELINE + drift_corr;
//				while (ASSR & _BV(OCR2AUB))
//					;
				displaymode = SHOW_TIME;
				return;
      }
    }
    if ((just_pressed & 0x4) || (pressed & 0x4)) {
      just_pressed = 0;
      if (mode == SET_DRIFTCORR) {
        drift_corr ++;
				if (drift_corr >= DRIFT_MAX)
					drift_corr = DRIFT_MIN;
				display_snum(6, drift_corr, 1);
      }
      if (pressed & 0x4) {
				tick();  // make a noise (wbp)
				delayms(75);
			}
    }
  }
}
#endif

#ifdef FEATURE_SECSMODE
// modes: FULL, DIAL1, DIAL2, AMPM (?), NONE
void show_secsmode(void) {
	switch (secsmode) {
		case 0: display_str(" full   "); break;
		case 1: display_str(" dial1  "); break;  // LF pattern, 6 steps, 1/sec
		case 2: display_str(" dial2  "); break;  // outer segments only, 8 steps, 1/8 minute interval
		case 3: display_str(" dial3  "); break;  // "cup" pattern, 12 steps, 5 second interval
		case 4: display_str(" dial4  "); break;  // fast radial pattern, 6 steps, 1/6 second interval
		case 5: display_str(" am-pm  "); break;
		case 6: display_str(" am-pm. "); display[7] |= 0x1; break;
		case 7: display_str(" none   "); break;
		case 8: display_str(" none.  "); display[6] |= 0x1; break;
		case 9: display_str(" fullgps"); display[8] |= 0x1; break;  // full with GPS update shown
	}
}

void set_secsmode(void) {
  uint8_t mode = SHOW_MENU;
  timeoutcounter = INACTIVITYTIMEOUT;
  while (1) {
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (just_pressed & 0x1) { // mode change
      return;
    }
    if (just_pressed & 0x2) {
      just_pressed = 0;
      if (mode == SHOW_MENU) {
				// start!
				mode = SET_SECSMODE;
				// display seconds mode
				show_secsmode();
      } else { 
				eeprom_update_byte((uint8_t *)EE_SECSMODE, secsmode);
				displaymode = SHOW_TIME;
				return;
      }
    }
    if ((just_pressed & 0x4) || (pressed & 0x4)) {
      just_pressed = 0;
      if (mode == SET_SECSMODE) {
        secsmode ++;
				if (secsmode > 9)  // 06may13/wbp
					secsmode = 0;
				show_secsmode();
      }
      if (pressed & 0x4) {  // adjust held down
				tick();  // make a noise (wbp)
				delayms(100);
			}
    }
  }
}
#endif

#ifdef FEATURE_SETSNOOZE
void set_snoozetime(void) {
  uint8_t mode = SHOW_MENU;
  uint8_t snooze;

  timeoutcounter = INACTIVITYTIMEOUT;
  snooze = eeprom_read_byte((uint8_t *)EE_SNOOZE);

  while (1) {
    if (just_pressed || pressed) {
      timeoutcounter = INACTIVITYTIMEOUT;
      // timeout w/no buttons pressed after 3 seconds?
    } else if (!timeoutcounter) {
      //timed out!
      displaymode = SHOW_TIME;     
      return;
    }
    if (just_pressed & 0x1) { // mode change
      return;
    }
    if (just_pressed & 0x2) { // button 2 - select

      just_pressed = 0;
      if (mode == SHOW_MENU) {
				// start!
				mode = SET_SNOOZETIME;
				// display snooze
				display_str("   minut");
				display_num(1, snooze, 1);
      } else { 
				display_str(" saving ");
				eeprom_update_byte((uint8_t *)EE_SNOOZE, snooze);
				displaymode = SHOW_TIME;
				return;
      }
    }
    if ((just_pressed & 0x4) || (pressed & 0x4)) { // button 3 - increment
      just_pressed = 0;
      if (mode == SET_SNOOZETIME) {
        snooze ++;
				if (snooze > 99)
					snooze = 0;
				display_num(1, snooze, 1);
      }
      if (pressed & 0x4) {
				tick();  // make a noise (wbp)
				delayms(75);
			}
    }
  }
}
#endif

/**************************** RTC & ALARM *****************************/
void clock_init(void) {

  // we store the time in EEPROM when switching from power modes so its
  // reasonable to start with whats in memory

  date_y = eeprom_read_byte((uint8_t *)EE_YEAR) % 100;
  date_m = eeprom_read_byte((uint8_t *)EE_MONTH) % 13;
  date_d = eeprom_read_byte((uint8_t *)EE_DAY) % 32;
	// display restored date (wbp)
	display_date(DATE);
	_delay_ms(500);  // wait a bit without unblocking...

  time_h = eeprom_read_byte((uint8_t *)EE_HOUR) % 24;
  time_m = eeprom_read_byte((uint8_t *)EE_MIN) % 60;
  time_s = eeprom_read_byte((uint8_t *)EE_SEC) % 60;
	// display restored time (wbp)
  display_time(time_h, time_m, time_s);
	_delay_ms(500);  // wait a bit without unblocking...

  /*
    // if you're debugging, having the makefile set the right
    // time automatically will be very handy. Otherwise don't use this
  time_h = TIMEHOUR;
  time_m = TIMEMIN;
  time_s = TIMESEC + 10;
  */

  // Set up the stored alarm time and date
  alarm_m = eeprom_read_byte((uint8_t *)EE_ALARM_MIN) % 60;
  alarm_h = eeprom_read_byte((uint8_t *)EE_ALARM_HOUR) % 24;

  restored = 1;

#ifdef FEATURE_DRIFTCORR
  drift_corr = eeprom_read_byte((uint8_t *)EE_DRIFTCORR);
//  if (drift_corr > DRIFT_MAX || drift_corr < -DRIFT_MIN) {  OOPS!
  if (drift_corr > DRIFT_MAX || drift_corr < DRIFT_MIN) {
    drift_corr = 0;
    eeprom_update_byte((uint8_t *)EE_DRIFTCORR, drift_corr);
  }
  /* 
   * Input is a (nominal) 32khz crystal.  Set:
   * - divider to 256
   * - mode to CTC
   * - comparitor to 127
   *
   * This will increment the counter at (nominally) 128Hz.  When it
   * compares to OCR2A it will reset the counter to 0 and raise an
   * interrupt so we can increment seconds.
   *
   * To correct drift we can adjust the comparitor to 127 +/-
   * correction.
   */
  // Turn on the RTC by selecting the external 32khz crystal
  ASSR = _BV(AS2); // use crystal

  TCNT2 = 0;
  OCR2A = DRIFT_BASELINE;		/* +/- drift correction */
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS22) | _BV(CS21);

  // enable interrupt
  TIMSK2 = _BV(OCIE2A);
#else
	// Turn on the RTC by selecting the external 32khz crystal
	// 32.768 / 128 = 256 which is exactly an 8-bit timer overflow
	ASSR |= _BV(AS2); // use crystal
	TCCR2B = _BV(CS22) | _BV(CS20); // div by 128
	// We will overflow once a second, and call an interrupt
	// enable interrupt
	TIMSK2 = _BV(TOIE2);  // Enable Timer/Counter2 Overflow Interrupt
#endif

  // enable all interrupts!
  sei();
}

// This turns on/off the alarm when the switch has been
// set. It also displays the alarm time
void setalarmstate(void) {
  if (ALARM_PIN & _BV(ALARM)) { 
    // Don't display the alarm/beep if we already have
    if  (!alarm_on) {
      // alarm on!
      alarm_on = 1;
      // reset snoozing
      snoozetimer = 0;
      // its not actually SHOW_SNOOZE but just anything but SHOW_TIME
			// check restored ???
      if(timeknown && displaymode == SHOW_TIME) //If we are in test mode, we would mess up
      {                           //testing of the display segments.
        // show the status on the VFD tube
        display_str("alarm on");
        displaymode = NONE;  // block time display
        delayms(1500);
        // show the current alarm time set
        display_alarm(alarm_h, alarm_m);
        delayms(1500);
        // after a second, go back to clock mode
        displaymode = SHOW_TIME;
      }
    }
  } else {
    if (alarm_on) {
      // turn off the alarm
      alarm_on = 0;
      snoozetimer = 0;
      if (alarming) {
				// if the alarm is going off, we should turn it off
				// and quiet the speaker
//				DEBUGP("alarm off");
				alarming = 0;
				TCCR1B &= ~_BV(CS11); // turn it off!
				PORTB |= _BV(SPK1) | _BV(SPK2);
      } 
    }
  }
}

// This will calculate leapyears, give it the year
// and it will return 1 (true) or 0 (false)
uint8_t leapyear(uint16_t y) {
  return ( (!(y % 4) && (y % 100)) || !(y % 400));
}


/**************************** SPEAKER *****************************/
// Set up the speaker to prepare for beeping!
void speaker_init(void) {
  // read the preferences for high/low volume
  volume = eeprom_read_byte((uint8_t *)EE_VOLUME);
  // We use the built-in fast PWM, 8 bit timer
  PORTB |= _BV(SPK1) | _BV(SPK2); 
  // Turn on PWM outputs for both pins
  TCCR1A = _BV(COM1B1) | _BV(COM1B0) | _BV(WGM11);
  if (volume) {
    TCCR1A |= _BV(COM1A1);
  } 
  TCCR1B = _BV(WGM13) | _BV(WGM12);
  // start at 4khz:  250 * 8 multiplier * 4000 = 8mhz
  ICR1 = 250;
  OCR1B = OCR1A = ICR1 / 2;
}

// This makes the speaker tick, it doesnt use PWM
// instead it just flicks the piezo
void tick(void) {
  TCCR1A = 0;
  TCCR1B = 0;
  // Send a pulse thru both pins, alternating
  SPK_PORT |= _BV(SPK1);
  SPK_PORT &= ~_BV(SPK2);
  delayms(10);
  SPK_PORT |= _BV(SPK2);
  SPK_PORT &= ~_BV(SPK1);
  delayms(10);
  // turn them both off
  SPK_PORT &= ~_BV(SPK1) & ~_BV(SPK2);
	// restore volume setting - 12oct12/wbp
  TCCR1A = _BV(COM1B1) | _BV(COM1B0) | _BV(WGM11);
  if (volume) {  // 12oct12/wbp
    TCCR1A |= _BV(COM1A1);
  } 
  TCCR1B = _BV(WGM13) | _BV(WGM12);
}

// We can play short beeps!
// 18nov11/wbp - shorten beep to 100 ms
void beep(uint16_t freq, uint8_t times) {
	beep_ms(freq, times, 100);
}
void beep_ms(uint16_t freq, uint8_t times, uint16_t ms) {
  // set the PWM output to match the desired frequency
  ICR1 = (F_CPU/8)/freq;
  // we want 50% duty cycle square wave
  OCR1A = OCR1B = ICR1/2;
  while (times--) {
    TCCR1B |= _BV(CS11); // turn it on!
    // beeps are 100ms long on
    _delay_ms(ms);
    TCCR1B &= ~_BV(CS11); // turn it off!
    PORTB &= ~_BV(SPK1) & ~_BV(SPK2);
    // beeps are 100ms long off
    _delay_ms(ms);
  }
  // turn speaker off
  PORTB &= ~_BV(SPK1) & ~_BV(SPK2);
}

#ifdef FEATURE_AUTODIM
/**************************** DIMMER ****************************/
void dimmer_init(void) {
  // Power for the photoresistor
  DIMMER_POWER_DDR |= _BV(DIMMER_POWER_PIN); 
  DIMMER_POWER_PORT |= _BV(DIMMER_POWER_PIN);

  ADCSRA |= _BV(ADPS2)| _BV(ADPS1); // Set ADC prescalar to 64 - 125KHz sample rate @ 8MHz F_CPU
	
  ADMUX |= _BV(REFS0);  // Set ADC reference to AVCC
  ADMUX |= _BV(DIMMER_SENSE_PIN);   // Set ADC input as ADC4 (PC4)
  DIDR0 |= _BV(DIMMER_SENSE_PIND); // Disable the digital imput buffer on the sense pin to save power.
  ADCSRA |= _BV(ADEN);  // Enable ADC
  ADCSRA |= _BV(ADIE);  // Enable ADC interrupt
}

// Start ADC conversion for dimmer
void dimmer_update(void) {
  if (autodim == AUTODIM_ON) 
    ADCSRA |= _BV(ADSC);
}

// Update brightness once ADC measurement completes
//SIGNAL(SIG_ADC) {  // 168 only
SIGNAL(ADC_vect) {  // 328p & 168
  uint8_t low, high;
  unsigned int val;
  if (autodim != AUTODIM_ON)
    return;
  // Read 2-byte value. Must read ADCL first because that locks the value.
  low = ADCL;
  high = ADCH;
  val = (high << 8) | low;
  #ifdef FEATURE_AUTODIM
  dimmer_adc = val;  // save for display
  #endif
  // Set brightness to a value between min & max based on light reading.
  if (val >= PHOTOCELL_DARK) {
    val = autodim_lo;
  } else if (val <= PHOTOCELL_LIGHT) {
    val = autodim_hi;
  } else {
    val = autodim_hi - (((unsigned long)(autodim_hi - autodim_lo)) *
      (val - PHOTOCELL_LIGHT)) / (PHOTOCELL_DARK - PHOTOCELL_LIGHT);
  }
	dimmer_lvl = val;  // save for display
  set_vfd_brightness(val);
}
#endif

/**************************** BOOST *****************************/

// We control the boost converter by changing the PWM output
// pins
void boost_init(uint8_t brightness) {
  set_vfd_brightness(brightness);
  // fast PWM, set OC0A (boost output pin) on match
  TCCR0A = _BV(WGM00) | _BV(WGM01);  
  // Use the fastest clock
  TCCR0B = _BV(CS00);
  TCCR0A |= _BV(COM0A1);
  TIMSK0 |= _BV(TOIE0); // turn on the interrupt for muxing
  sei();
}

void set_vfd_brightness(uint8_t brightness) {
	if (alarming)  // if alarming, flash display, even if snoozing...
//		brightness = (time_s % 2) ? BRITE_MIN : BRITE_MAX;
		brightness = (time_s % 2) ? 50 : BRITE_MAX;  // BRITE_MIN is too dim...
  // Set PWM value, don't set it so high that
  // we could damage the MAX chip or display
  if (brightness > BRITE_MAX)
    brightness = BRITE_MAX;
  // Or so low its not visible
#ifdef FEATURE_AUTODIM
  if (brightness < AUTO_BRITE_MIN)
    brightness = AUTO_BRITE_MIN;
#else
  if (brightness < BRITE_MIN)
    brightness = BRITE_MIN;
#endif
  if (OCR0A == brightness)
    return;
  OCR0A = brightness;
}

// Calculate day of the week - Sunday=1, Saturday=7  (non ISO)
uint8_t dotw(uint8_t year, uint8_t month, uint8_t day)
{
  uint16_t m, y;
	m = month;
	y = 2000 + year;
  if (m < 3)  {
    m += 12;
    y -= 1;
  }
	return (day + (2 * m) + (6 * (m+1)/10) + y + (y/4) - (y/100) + (y/400) + 1) % 7 + 1;
}

/**************************** DISPLAY *****************************/

char get_number(char b) {
	return(pgm_read_byte(numbertable_p + b));
}
char get_alpha(char b) {
	return(pgm_read_byte(alphatable_p + b - 'a'));
}

// We can display the current date!
void display_date(uint8_t style) {

  // This type is mm-dd-yy OR dd-mm-yy depending on our pref.
  if (style == DATE) {
    display[0] = 0;
    display[6] = display[3] = 0x02;     // put dashes between num

    if (region == REGION_US) {
      // mm-dd-yy
//      display[1] = get_number((date_m / 10));
//      display[2] = get_number((date_m % 10));
//      display[4] = get_number((date_d / 10));
//      display[5] = get_number((date_d % 10));
			display_num(1, date_m, 0);
			display_num(4, date_d, 0);
    } else {
      // dd-mm-yy
//      display[1] = get_number((date_d / 10));
//      display[2] = get_number((date_d % 10));
//      display[4] = get_number((date_m / 10));
//      display[5] = get_number((date_m % 10));
			display_num(1, date_d, 0);
			display_num(4, date_m, 0);
    }
    // the yy part is the same
//    display[7] = get_number((date_y / 10));
//    display[8] = get_number((date_y % 10));
		display_num(7, date_y, 0);
  } else if (style == DAY) {
    // This is more "Sunday June 21" style
    // Display the day first
    display[8] = display[7] = 0;
    switch (dotw(date_y, date_m, date_d)) {
    case 1:
      display_str("sunday"); break;
    case 2:
      display_str("monday"); break;
    case 3:
      display_str("tuesday"); break;
    case 4:
      display_str("wednesdy"); break;
    case 5:
      display_str("thursday"); break;
    case 6:
      display_str("friday"); break;
    case 7:
      display_str("saturday"); break;
    }
    
    // wait one seconds about
    delayms(1000);

    // Then display the month and date
    display[6] = display[5] = display[4] = 0;
    switch (date_m) {
    case 1:
      display_str("jan"); break;
    case 2:
      display_str("feb"); break;
    case 3:
      display_str("march"); break;
    case 4:
      display_str("april"); break;
    case 5:
      display_str("may"); break;
    case 6:
      display_str("june"); break;
    case 7:
      display_str("july"); break;
    case 8:
      display_str("augst"); break;
    case 9:
      display_str("sept"); break;
    case 10:
      display_str("octob"); break;
    case 11:
      display_str("novem"); break;
    case 12:
      display_str("decem"); break;
    }
//    display[7] = get_number((date_d / 10));
//    display[8] = get_number((date_d % 10));
		display_num(7, date_d, 0);

    // wait one second about
    delayms(1000);

		// display year
		display_str(" 2000   ");
//    display[3] = get_number((date_y / 100));  // testing (wbp)
//    display[4] = get_number((date_y / 10));
//    display[5] = get_number((date_y % 10));
		display_num(4, date_y, 0);
//    display[8] = display[7] = display[6] = display[5] = 0;
    
  }
}

// This displays a time on the clock
void display_time(uint8_t h, uint8_t m, uint8_t s) {

#ifdef FEATURE_SECSMODE
	if (displaymode == SHOW_TIME)  {  // secsmode only active if displaymode is SHOW_TIME
	uint16_t ss;
	 switch (secsmode)  {
		case 0:  // secs
			display_num(7, s, 0);
			break;
		case 1:   // dial 1 - by Larry Frank
			ss = s%6*2;  // 6 steps, 1 every second
			display[7] = pgm_read_byte(dialsegs1_p + ss + 1);
			display[8] = pgm_read_byte(dialsegs1_p + ss);
			break;
		case 2:  // dial 2 - outer bars, with blink
//			ss = s*10/75*2;  // 8 steps, 1 every 7.5 secs
			ss = s%10*2;  // 10 steps, 1 every second
			display[7] = pgm_read_byte(dialsegs2_p + ss + 1);
			display[8] = pgm_read_byte(dialsegs2_p + ss);
			if ((s&1) == 1)  { // blink center bar
				display[7] |= 0x02;  // bar is bit 2
				display[8] |= 0x02;  // bar is bit 2
			}
			break;
		case 3:  // dial 3 - snake pattern
			ss = s%12*2;  // 12 steps, 1 a second
			display[7] = pgm_read_byte(dialsegs3_p + ss + 1);
			display[8] = pgm_read_byte(dialsegs3_p + ss);
			break;
//		case 4:  // dial 4 is a fast pattern handled elsewhere
//			break;
		case 5:   // AM/PM
		case 6:   // AM/PM b
			if (h<12)
				display[7] = get_alpha('a');
			else
				display[7] = get_alpha('p');
			display[8] = get_alpha('m');
			if (secsmode == 6)  // blink decimal point
				display[8] |= (s&1);
			break;
		case 7:   // none
		case 8:   // none b
			display[7] = 0;
			display[8] = 0;
			if (secsmode == 8)  // blink decimal point
				display[8] |= (s&1);
			break;
		case 9:  // use dp to show GPS status - solid if good signal
			display_num(7, s, 0);  // display seconds (clears dp)
			if (gpsEnabled) {
				if (gpsTimeout<gpsTimeout1) {
					display[8] |= 1;  // solid dp indicates good gps signal reception
				}
//				else if (gpsTimeout<gpsTimeout2) {
//					display[8] &= 0;  // no dp indicates no gps signal reception
//				}
				else if (gpsTimeout>gpsTimeout2) {
					display[8] |= (s&1);  // no gps signal for a long time - flash decimal point
				}
			}
			break;
		}
	 }
	else  // DISPLAY_MODE not SHOW_TIME
#endif
	 display_num(7, s, 0);  // display seconds
	
  display[6] = 0;  // blank
	display_num(4, m, 0);  // minutes
  display[3] = 0;  // blank

  // check euro (24h) or US (12h) style time
  if (region == REGION_US) {
    display[2] =  get_number(( (((h+11)%12)+1) % 10));
    if ((((h+11)%12)+1) / 10 == 0 ) {
      display[1] =  0;
    } else {
      display[1] =  get_number(1);
    }
    // We use the '*' as an am/pm notice
    if (h >= 12)
      display[0] |= 0x1;  // 'pm' notice
    else 
      display[0] &= ~0x1;  // 'pm' notice
  } else {  // 24 hour time
//    display[2] =  get_number(( (h%24) % 10));
//    display[1] =  get_number(( (h%24) / 10));
		display_num(1, h, 0);
  }
}

// Kinda like display_time but just hours and minutes
void display_alarm(uint8_t h, uint8_t m){ 
  display[8] = 0;
  display[7] = 0;
  display[6] = 0;
//  display[5] = get_number((m % 10));
//  display[4] = get_number((m / 10)); 
	display_num(4, m, 0);
  display[3] = 0;

  // check euro or US style time
  if (region == REGION_US) {
    if (h >= 12) {
      display[0] |= 0x1;  // 'pm' notice
      display[7] = get_alpha('p');
    } else {
      display[7] = get_alpha('a');
      display[0] &= ~0x1;  // 'am' notice
    }
    display[8] = get_alpha('m');

    display[2] =  get_number(( (((h+11)%12)+1) % 10));
    if ((((h+11)%12)+1) / 10 == 0 ) {
      display[1] =  0;
    } else {
      display[1] =  get_number(1);
    }
  } else {
//    display[2] =  get_number(( (((h+23)%24)+1) % 10));
//    display[1] =  get_number(( (((h+23)%24)+1) / 10));
		display_num(1, (((h+23)%24)+1), 0);
  }
}

// Kinda like display_time but just hours and minutes allows negative hours.
void display_timezone(int8_t h, uint8_t m){ 
  display[8] = get_alpha('c');
  display[7] = get_alpha('t');
  display[6] = get_alpha('u');
//  display[5] = get_number((m % 10));
//  display[4] = get_number((m / 10)); 
	display_num(4, m, 0);
  display[3] = 0;
//  display[2] = get_number((abs(h) % 10));
//  display[1] = get_number((abs(h) / 10));
	display_num(1, abs(h), 0);
  // We use the '-' as a negative sign
  if (h >= 0)
    display[0] &= ~0x2;  // positive numbers, implicit sign
  else 
    display[0] |= 0x2;  // negative numbers, display negative sign
}

// display words (menus, prompts, etc)
void display_str(char *s) {
  uint8_t i;
  // don't use the lefthand dot/slash digit
  display[0] = 0;
  // up to 8 characters
  for (i=1; i<9; i++) {
    // check for null-termination
    if (s[i-1] == 0)
      return;
    // Numbers and leters are looked up in the font table!
    if ((s[i-1] >= 'a') && (s[i-1] <= 'z')) {
      display[i] =  get_alpha(s[i-1]);
    } else if ((s[i-1] >= '0') && (s[i-1] <= '9')) {
      display[i] =  get_number(s[i-1] - '0');
    } else {
      display[i] = 0;      // spaces and other stuff are ignored :(
    }
  }
}

/************************* LOW LEVEL DISPLAY ************************/

// Setup SPI
void vfd_init(void) {
  SPCR  = _BV(SPE) | _BV(MSTR) | _BV(SPR0);
}

// This changes and updates the display
// We use the digit/segment table to determine which
// pins on the MAX6921 to turn on
//void setdisplay(uint8_t digit, uint8_t segments) {  // wbp
void setdisplay(uint8_t digit) {
	if (digit > lastdigit)
		digit = lastdigit;  // hack to display last digit twice
  uint8_t segments = display[digit];
  uint32_t d = 0;  // we only need 20 bits but 32 will do
  uint8_t i;
  // Set the digit selection pin
  d |= _BV(pgm_read_byte(digittable_p + digit));
  // Set the individual segments for this digit
  for (i=0; i<8; i++) {
    if (segments & _BV(i)) {
      t = 1;
      t <<= pgm_read_byte(segmenttable_p + i);
      d |= t;
    }
  }
  // Shift the data out to the display
  vfd_send(d);
}

// send raw data to display, its pretty straightforward. Just send 32 bits via SPI
// the bottom 20 define the segments
void vfd_send(uint32_t d) {
  // send lowest 20 bits
  cli();       // to prevent flicker we turn off interrupts
  spi_xfer(d >> 16);
  spi_xfer(d >> 8);
  spi_xfer(d);

  // latch data
  VFDLOAD_PORT |= _BV(VFDLOAD);
  VFDLOAD_PORT &= ~_BV(VFDLOAD);
  sei();
}

// Send 1 byte via SPI
void spi_xfer(uint8_t c) {
  SPDR = c;
  while (! (SPSR & _BV(SPIF)));
}

#ifdef FEATURE_GPS
//GPS serial data handling functions:

//Check to see if there is any serial data.
uint8_t gpsDataReady(void) {
  return (UCSR0A & _BV(RXC0));
}

// get data from gps and update the clock (if possible)
//$GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68\r\n
// 0         1         2         3         4         5         6         7
// 01234567890123456789012345678901234567890123456789012345678901234567890
//    0     1   2    3    4     5    6   7     8      9     10  11 12
//
//  225446       Time of fix hhmmss 22:54:46 UTC
//  A            Navigation receiver warning A = OK, V = warning
//  4916.45,N    Latitude 49 deg. 16.45 min North
//  12311.12,W   Longitude 123 deg. 11.12 min West
//  000.5        Speed over ground, Knots
//  054.7        Course Made Good, True
//  191194       Date of fix ddmmyy 19 November 1994
//  020.3,E      Magnetic variation 20.3 deg East
//  *68          mandatory checksum
void getGPSdata(void) {
//  uint8_t intOldHr = 0;
//  uint8_t intOldMin = 0;
//  uint8_t intOldSec = 0;
  char charReceived = UDR0;  // get a byte from the port
	uint8_t bufflen = strlen(gpsBuffer);
  //If the buffer has not been started, check for '$'
  if ( ( bufflen == 0 ) &&  ( '$' != charReceived ) )
		return;  // wait for start of next sentence from GPS
	if ( bufflen < (GPSBUFFERSIZE - 1) ) {  // is there room left? (allow room for null term)
		if ( '\r' != charReceived ) {  // end of sentence?
			strncat(gpsBuffer, &charReceived, 1);  // add char to buffer
			return;
		}
		else
			strncat(gpsBuffer, "*", 1);  // mark end of buffer just in case
		// end of sentence - is this the message we are looking for?
		if ( strncmp( gpsBuffer, "$GPRMC,", 7 ) == 0 ) {  
			//Calculate checksum from the received data
			gpsPtr = &gpsBuffer[1];  // start at the "G"
			gpsCheck1 = 0;  // init collector
			 /* Loop through entire string, XORing each character to the next */
			while (*gpsPtr != '*')  // count all the bytes up to the asterisk
			{
				gpsCheck1 ^= *gpsPtr;
				gpsPtr++;
			}
			// now get the checksum from the string itself, which is in hex
			uint8_t chk1, chk2;
			chk1 = *(gpsPtr+1);
			chk2 = *(gpsPtr+2);
			if (chk1 > '9') 
				chk1 = chk1 - 55;  // convert 'A-F' to 10-15
			else
				chk1 = chk1 - 48;  // convert '0-9' to 0-9
			if (chk2 > '9') 
				chk2 = chk2 - 55;  // convert 'A-F' to 10-15
			else
				chk2 = chk2 - 48;  // convert '0-9' to 0-9
			gpsCheck2 = (chk1 * 16)  + chk2;
			if (gpsCheck1 == gpsCheck2) {  // if checksums match, process the data
				//Find the first comma:
				gpsPtr = strchr( gpsBuffer, ',');
				//Copy the section of memory in the buffer that contains the time.
				memcpy( gpsTime, gpsPtr + 1, 6 );
				gpsTime[6] = 0;  //add a null character to the end of the time string.
				gpsPtr = strchr( gpsPtr + 1, ',');  // find the next comma
				memcpy( gpsFixStat, gpsPtr + 1, 1 );  // copy fix status
				if (gpsFixStat[0] == 'A') {  // if data valid, update time & date
//					gpsTimeout = 0;  // reset gps timeout counter
					gpsPtr = strchr( gpsPtr + 1, ',');  // find the next comma
					memcpy( gpsLat, gpsPtr + 1, 4 );  // copy Latitude ddmm
					memcpy( gpsLat + 4, gpsPtr + 6, 2 );  // copy Latitude ff
					gpsPtr = strchr( gpsPtr + 1, ',');  // find the next comma
					memcpy( gpsLatH, gpsPtr + 1, 1 );  // copy Latitude Hemisphere
					gpsPtr = strchr( gpsPtr + 1, ',');  // find the next comma
					memcpy( gpsLong, gpsPtr + 1 , 5 );  // copy Longitude dddmm
					memcpy( gpsLong + 5, gpsPtr + 7 , 2 );  // copy Longitude ff
					gpsPtr = strchr( gpsPtr + 1, ',');  // find the next comma
					memcpy( gpsLongH, gpsPtr + 1 ,1 );  // copy Longitude Hemisphere
					//Find three more commas to get the date:
					for ( int i = 0; i < 3; i++ ) {
						gpsPtr = strchr( gpsPtr + 1, ',');
					}
					//Copy the section of memory in the buffer that contains the date.
					memcpy( gpsDate, gpsPtr + 1, 6 );
					gpsDate[6] = 0;  //add a null character to the end of the date string.
					getGPStime();  // data is ready, update the clock
				}
			}
		}  // if "$GPRMC"
	}  // if space left in buffer
	// either buffer is full, or the message has been processed. reset buffer for next message
	memset( gpsBuffer, 0, GPSBUFFERSIZE );
}  // getGPSdata

// set time from gps data gathered previously
void getGPStime(void) {
	// this is only called when GPRMC message is received, the checksums match, and Status = 'A' 
//	if ( restored && ( PROGRAMMING_YEAR <= ( ( (gpsDate[4] - '0') * 10 ) ) + (gpsDate[5] - '0') ) ) {
	if ( restored ) {  // clock running?
		// only accept GPS data if 2 messages in sequence have same Year, Month, Day, Hour, and Minute
		// this catches corrupt message strings, but also skips a few messages
		// the GPS emits a GPRMC message once a second, so skipping a message now and then is fine
		if ((strncmp(gpsDate, gpsPrevDate, 6) == 0) && (strncmp(gpsTime,gpsPrevTime,4) == 0)) {
			gpsTimeout = 0;  // good signal has been received from GPS
  		//Change the time:
	  	setgpstime(gpsTime);
		  //Change the date:
		  setgpsdate(gpsDate);
		  //Gussy up the time and date, make the numbers come out right:
		  fix_time();
		}
		strncpy(gpsPrevDate, gpsDate, 6);  // save gps date & time for next check
		strncpy(gpsPrevTime, gpsTime, 6);
		
// //Get the 'new' value of the time and the alarm time in minutes:
// // this code is broken - it causes alarm at midnight
// uint16_t newTime = ((time_h * 60) + time_m);
// uint16_t timeAlarm = ((alarm_h*60) + alarm_m);
// //If midnight happened between the old time and the new time
// // and we did not just go back in time...
// //		if ( ( 0 > (int16_t)( newTime - oldTime ) ) 
// if ( ( newTime < oldTime ) 
		 // && ( (newTime + 1440) >= oldTime )
		 // && ( abs( newTime + 1440 - oldTime ) < abs( newTime - oldTime ) ) ) {
	// newTime += 1440;  // just hit midnight, add 1 day to new time for next step
// }
// if ( (oldTime<timeAlarm) && (newTime>=timeAlarm) && alarm_on ) {
	// start_alarm();  // GPS update skipped the alarm time, start it beeping now...
// }

 }
	
}

//Set the time with a string taken from GPS data:
void setgpstime(char* str) {
  uint8_t intTempHr = 0;
  uint8_t intTempMin = 0;
  uint8_t intTempSec = 0;
  intTempHr = (str[0] - '0') * 10;
  intTempHr = intTempHr + (str[1] - '0');
  intTempMin = (str[2] - '0') * 10;
  intTempMin = intTempMin + (str[3] - '0');
  intTempSec = (str[4] - '0') * 10;
  intTempSec = intTempSec + (str[5] - '0');
#ifdef FEATURE_WmDST
  time_h = intTempHr + intTimeZoneHour + dst_offset;
#else
  time_h = intTempHr + intTimeZoneHour;
#endif
  //If the time zone offset is negative, then subtract minutes
  if ( 0 > intTimeZoneHour )
    time_m = intTempMin - intTimeZoneMin;
  else
    time_m = intTempMin + intTimeZoneMin;
  time_s = intTempSec;
  timeknown = 1;  // time is now known
}

//Set the date with a string taken from GPS data:
void setgpsdate(char* str) {
  uint8_t intTempDay = 0;
  uint8_t intTempMon = 0;
  uint8_t intTempYr = 0;
  intTempDay = (str[0] - '0') * 10;
  intTempDay = intTempDay + (str[1] - '0');
  intTempMon = (str[2] - '0') * 10;
  intTempMon = intTempMon + (str[3] - '0');
  intTempYr = (str[4] - '0') * 10;
  intTempYr = intTempYr + (str[5] - '0');
//  restored = 0;  // was used to prevent saving time in ee when going to sleep (why???)
  date_d = intTempDay;
  date_m = intTempMon;
  date_y = intTempYr;
  dateknown = 1;  // date is now known
}
#endif

//Starts the alarm beeping
void start_alarm()  {
//  DEBUGP("alarm on!");
  alarming = 1;
  snoozetimer = 0;
	alarm_cycle = 200;  // 20 second initial cycle
	alarm_count = 0;  // reset cycle count
	beep_cycle = 2;  // start with single beep
	alarmdiv = alarm_timer = 0;  // start beeping now
}

//Checks the alarm against the passed time.
void check_alarm(uint8_t h, uint8_t m, uint8_t s) {
  if (alarm_on && (alarm_h == h) && (alarm_m == m) && (0 == s)) {
		start_alarm();
  }
}


//Fixes the time variables whenever time is changed
void fix_time(void) {
  // a minute!
  if (time_s >= 60) {
    time_s = time_s - 60;
    time_m++;
  }
  // If someone decides to make offset seconds with a negative number...
  if (time_s < 0) {
    time_s =  60 + time_s;
    time_m--;
  }

  // an hour...
  if (time_m >= 60) {
    time_m = time_m - 60;
    time_h++; 
  }
  // When offsets create negative minutes...
  if (time_m < 0) {
    time_m = 60 + time_m;
    time_h--; 
		// hopefully this does not happen very often
  }

#ifdef FEATURE_GPS
	if ((time_s == 0) && (time_m == 0) && (gpsEnabled == GPS_OFF))  {  // on the hour?
#else
	if ((time_s == 0) && (time_m == 0))  {  // on the hour?
#endif
    // let's write the time to the EEPROM - done once per hour but only if GPS is off
		// at 100,000 write cycles this should be good for about 4100 days...
    eeprom_update_byte((uint8_t *)EE_HOUR, time_h);
    eeprom_update_byte((uint8_t *)EE_MIN, time_m);
	}

  // a day....
  if (time_h >= 24) {
    time_h = time_h - 24;
    date_d++;
    eeprom_update_byte((uint8_t *)EE_DAY, date_d);
  }
  // When offsets create negative hours...
  if (time_h < 0) {
    time_h = 24 + time_h;
    date_d--;
    eeprom_update_byte((uint8_t *)EE_DAY, date_d);
  }
  
  //if (! sleepmode) {
  //  uart_putw_dec(time_h);
  //  uart_putchar(':');
  //  uart_putw_dec(time_m);
  //  uart_putchar(':');
  //  uart_putw_dec(time_s);
  //  putstring_nl("");
  //}
  
  // a full month!
  // we check the leapyear and date to verify when it's time to roll over months
  if ((date_d > 31) ||
      ((date_d == 31) && ((date_m == 4)||(date_m == 6)||(date_m == 9)||(date_m == 11))) ||
      ((date_d == 30) && (date_m == 2)) ||
      ((date_d == 29) && (date_m == 2) && !leapyear(2000+date_y))) {
    date_d = 1;
    date_m++;
    eeprom_update_byte((uint8_t *)EE_MONTH, date_m);
  }
  // When offsets create negative days...
  if (date_d < 1) {
    //Find which month we are going back to:
    switch (date_m) {
      case 1: //January -> December
      case 2: //February -> January
      case 4: //April -> March
      case 6: //June -> May
      case 8: //August -> July
      case 9: //September -> August
      case 11: //November -> October
        date_d = 31 + date_d;
        date_m--;
        break;
      case 5: //May -> April
      case 7: //July -> June
      case 10: //October -> September
      case 12: //December -> November
        date_d = 30 + date_d;
        date_m--;
        break;
      case 3: //March -> February, the fun case
        //If we are in a leapyear, February is 29 days long...
        if ( leapyear(2000+date_y) )
          date_d = 29 + date_d;
        else //otherwise, it is 28 days long...
          date_d = 28 + date_d;
        date_m--;
        break;
      default:
        date_d = 1;
        break;
    }
    eeprom_update_byte((uint8_t *)EE_MONTH, date_m);
  }
  
  // HAPPY NEW YEAR!
  if (date_m > 12) {
    date_y++;
    date_m = 1;
    eeprom_update_byte((uint8_t *)EE_YEAR, date_y);
  }
  //This takes away the years and is cheaper than any cream you can buy...
  if (date_m < 1) {
    date_m = 12 + date_m;
    date_y--;
    eeprom_update_byte((uint8_t *)EE_YEAR, date_y);
    eeprom_update_byte((uint8_t *)EE_MONTH, date_m);
  }

#ifdef FEATURE_WmDST
	if (time_s == 0) {  // done once a minute - overkill but it will update for DST when clock is booted or time is adjusted
    if (dst_mode == DST_AUTO)  //Check daylight saving time.
			setDSToffset();  // set DST offset based on DST rules
//		else if(dst_mode == DST_ON) 
//			dst_offset = 1;  // fixed offset
//		else
//			dst_offset = 0;  // no offset
		if ((time_m == 0) && (time_h == 0))  // Midnight?
			dst_update = DST_YES;  // Reset DST Update flag at midnight
	}
#endif

}


#ifdef FEATURE_WmDST
long yearSeconds(uint8_t yr, uint8_t mo, uint8_t da, uint8_t h, uint8_t m, uint8_t s)
{
  long dn = tmDays[(mo-1)]+da;  // # days so far if not leap year
	yr += 2000;  // need 4 digit year
  if ((yr % 4 == 0 && yr % 100 != 0) || yr % 400 == 0)  // if leap year
    dn ++;  // add 1 day
  dn = dn * 86400 + (long)h*3600 + (long)m*60 + s;
  return dn;
} 

long DSTseconds(uint8_t year, uint8_t month, uint8_t doftw, uint8_t week, uint8_t hour)
{
	uint8_t dom = mDays[month-1];  //06nov12/wbp
	if ( (month == 2) && (year%4 == 0) )
		dom ++;  // february has 29 days this year
	uint8_t dow = dotw(year, month, 1);  // DOW for 1st day of month for DST event
	int8_t day = doftw - dow;  // number of days until 1st dotw in given month
		if (day<1)  day += 7;  // make sure it's positive 
  if (doftw >= dow)
    day = doftw - dow;
  else
    day = doftw + 7 - dow;
	day = 1 + day + (week-1)*7;  // date of dotw for this year
	while (day > dom)  // handles "last DOW" case
		day -= 7;
  return yearSeconds(year,month,day,hour,0,0);  // seconds til DST event this year
}

// adjust dst offset and save values in ee prom
void adjDSToffset(uint8_t offset)
{
	int8_t adj = offset - dst_offset;
	if ((dst_offset != offset) && (dst_update == DST_YES)) {
		if (adj>0) { // add to hour?
			beep(440,1);
			beep(880,1);
		}
		else { // subtract
			beep(880,1);
			beep(440,1);
		}
		dst_offset = offset;
		time_h += adj;  // if this is the first time, bump the hour
		eeprom_update_byte((uint8_t *)EE_DSTOFFSET, offset);  // remember setting for power up
		eeprom_update_byte((uint8_t *)EE_HOUR, time_h);    
		dst_update = DST_NO;  // OK, it's done, don't do it again today
	}
}

// DST Rules: Start(month, dotw, n, hour), End(month, dotw, n, hour), Offset
// DOTW is Day of the Week.  1=Sunday, 7=Saturday
// N is which occurrence of DOTW
// Current US Rules: March, Sunday, 2nd, 2am, November, Sunday, 1st, 2 am, 1 hour
// 		3,1,2,2,  11,1,1,2,  1
void setDSToffset()
{
	uint8_t month1 = dst_rules[0];
	uint8_t dotw1 = dst_rules[1];
	uint8_t n1 = dst_rules[2];  // nth dotw
	uint8_t hour1 = dst_rules[3];
	uint8_t month2 = dst_rules[4];
	uint8_t dotw2 = dst_rules[5];
	uint8_t n2 = dst_rules[6];
	uint8_t hour2 = dst_rules[7];
	uint8_t offset = dst_rules[8];
	// if current time & date is at or past the first DST rule and before the second, set dst_offset, otherwise reset dst_offset
  long dst_start = DSTseconds(date_y, month1, dotw1, n1, hour1);  // seconds til DST starts this year
  long dst_end = DSTseconds(date_y, month2, dotw2, n2, hour2);  // seconds til DST ends this year
  long seconds_now = yearSeconds(date_y, date_m, date_d, time_h, time_m, time_s);  //time now in seconds this year
	if (dst_end>dst_start) {  // dst end later than start - northern hemisphere
		if ((seconds_now >= dst_start) && (seconds_now < dst_end)) { // spring ahead
			adjDSToffset(offset);
		}
		else { // fall back
			adjDSToffset(0);
		}
	}
	else {  // dst start later in year than dst end - southern hemisphere
		if ((seconds_now >= dst_start) || (seconds_now < dst_end))  // fall ahead
			adjDSToffset(offset);
		else  // spring back
			adjDSToffset(0);
	}
}
#endif

#ifdef FEATURE_TESTMODE
void testmode(uint8_t force) {
  uint8_t seconds = time_s;
  uint8_t alarm_state = 0;
  uint8_t testdigit=0;
  uint8_t testvalue=0;
  uint8_t testexit=5;
  //uint8_t dim_on, dim_status;
  uint8_t i;
  #ifdef FEATURE_AUTODIM
  uint8_t j=0;
  uint16_t k;
  uint8_t blevel=0xFF;
  #endif
  
  if(!force)
  {
    if ((PIND & _BV(BUTTON1))) {
      return;
    }
    if((PIND & _BV(BUTTON3))) {
        return;
    }
  }
  beep(2000,1);
  beep(3000,1);
  beep(4000,1);
  while (!(PIND & _BV(BUTTON1)));
  while (!(PIND & _BV(BUTTON3)));
  for(i=0;i<9;i++)
    display[i] = 0;
  displaymode = TESTMODE;
  while(1) {
    kickthedog();
    
    #ifdef FEATURE_AUTODIM
    if(j!=0)
    {
      k=dimmer_adc;
      display[8] = get_number((k % 10));
      k/=10;
      display[7] = get_number((k % 10));
      k/=10;
      display[6] = get_number((k % 10));
      display[5] = get_number((k / 10));
    }
    #endif
    if(just_pressed&1)
    {
      beep(2000,1);
      just_pressed &= ~1;
      #ifdef FEATURE_AUTODIM
      if(j==0)
      {
      #endif
        if((testdigit==0))
        {
          if(display[0]==0)
            display[0]=1;
          else if (display[0]==1)
            display[0]=2;
          else
          {
            display[0]=0;
            testdigit++;
            display[testdigit]=1;
          }
        }
        else
        {
          display[testdigit]<<=1;
          if(display[testdigit]==0)
          {
            testdigit++;
            if(testdigit==9)
              testdigit=0;
            else
              display[testdigit]=1;
          }
        }
      #ifdef FEATURE_AUTODIM
      }
      #endif
        
    }
    if(just_pressed&2)
    {
      beep(2500,1);
      just_pressed &= ~2;
      #ifdef FEATURE_AUTODIM
      if(!j)
      {
        blevel = bright_level;
        bright_level = 0;  // turn display off ???
        dimmer_update();
        j = 1;
        //dimmer_update();
      }
      #endif
    }
    if(just_pressed&4)
    {
      beep(3000,1);
      just_pressed &= ~4;
      #ifdef FEATURE_AUTODIM
      if(j)
      {
        if(blevel!=0xFF)
        {
          bright_level = blevel;
          set_vfd_brightness(blevel);
        }
        j=0;
        for(i=0;i<9;i++)
          display[i] = 0;
        //set_vfd_brightness(bright_level);
      }
      #endif
      
    }
    if(seconds != time_s)
    {
      time_s = seconds;
      tick();
      if(!(PIND & _BV(BUTTON1))) {
        if(!(PIND & _BV(BUTTON3))) {
          testexit--;
          switch(testexit)
          {
            case 4:
              testvalue = display[testdigit];
              display_str("e        ");
              break;
            case 3:
              display_str("ex");
              break;
            case 2:
              display_str("exi");
              break;
            case 1:
              display_str("exit");
              break;
            default:
              display_str("exiting");
              break;
          }
          if(testexit==0)
          {
            while (!(PIND & _BV(BUTTON1)));
            while (!(PIND & _BV(BUTTON3)));
            beep(4000,1);
            beep(3000,1);
            beep(2000,1);
            #ifdef FEATURE_AUTODIM
            if(blevel!=0xFF)
            {
              if(blevel)
              {
                bright_level = blevel;
                set_vfd_brightness(blevel);
              }
              else
              {
                dimmer_update();
              }
            }
            #endif
            return;
          }
        }
        else
        {
          if(testexit<5)
          {
            for(i=0;i<9;i++)
              display[i] = 0;
            display[testdigit] = testvalue;
          }
          testexit=5;
        }
      }
      else
      {
        if(testexit<5)
        {
          for(i=0;i<9;i++)
            display[i] = 0;
          display[testdigit] = testvalue;
        }
        testexit=5;
      }
    }
    if(alarm_state != alarm_on)
    {
      alarm_state = alarm_on;
      beep(4000,1 + alarm_on);
    }
  }
}
#endif
