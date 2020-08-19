/*******************************************************************************
 Mini Ultra Pro Sleep RTC Alarm
 Version: 0.10
 Date: 20-02-2017
 Company: Rocket Scream Electronics
 Author: Lim Phang Moh
 Website: www.rocketscream.com

 An example code demonstrating the ultra low sleep current capability of the 
 Mini Ultra Pro while running the on-chip built-in RTC. Example puts the Mini
 Ultra Pro into standby mode (sleep) while RTC is allowed to run in the 
 background. Alarm is set to wake the processor on the 15 s of a minute.
 This results in sleep period of 1 minute. The LED on pin 13 is set to toggle
 on every alarm match state. In sleep state with the built-in LED turn off, 
 current measured is approximately 21.0 uA.

 Requirement:
 1. Mini Ultra Pro board powered by a single cell Li-Ion/Pol 3.7V battery.

 Important Note:
 1. Upon using the SerialFlash.sleep() function, the only function that the 
 serial flash chip response to is SerialFlash.wakeup(). 

 Revision Description
 ======== ===========
 0.10 Initial public release.
*******************************************************************************/

#include <SerialFlash.h>
#include <RTCZero.h>
#include <SPI.h>
//#include <RH_RF95.h>
//#include <RH_RF69.h>

// ***** CONSTANTS *****
const int radioDio0 = 2;
const int flashChipSelect = 4;
const int radioChipSelect = 5;

// Choose correct on-board radio
//RH_RF95 radio(radioChipSelect, radioDio0);
//RH_RF69 radio(radioChipSelect, radioDio0);
RTCZero rtc;

/* Change these values to set the current initial time */
const uint8_t seconds = 00;
const uint8_t minutes = 00;
const uint8_t hours = 10;

/* Change these values to set the current initial date */
const uint8_t day = 20;
const uint8_t month = 2;
const uint8_t year = 17;

// ***** VARIABLES *****
unsigned char pinNumber;

void setup()
{
 // Switch unused pins as input and enabled built-in pullup 
 for (pinNumber = 0; pinNumber < 23; pinNumber++){
 pinMode(pinNumber, INPUT_PULLUP);
 }

 for (pinNumber = 32; pinNumber < 42; pinNumber++){
 pinMode(pinNumber, INPUT_PULLUP);
 }

 pinMode(25, INPUT_PULLUP);
 pinMode(26, INPUT_PULLUP);

 //if (!radio.init()){}
 //radio.sleep();
 
 SerialFlash.begin(flashChipSelect);
 SerialFlash.sleep();

 pinMode(LED_BUILTIN, OUTPUT);
 
 digitalWrite(LED_BUILTIN, HIGH); // Why is this pulled high? Is it just to display when the timer interrupt goes off?

 // ***** IMPORTANT DELAY FOR CODE UPLOAD BEFORE USB PORT DETACH DURING SLEEP *****
 delay(15000);

 //***** UNCOMMENT FOR MINI ULTRA PRO VERSION 0V30 & ABOVE IF PINS ARE CONNECTED TO RADIO ****
 /*pinMode(6, INPUT);
 pinMode(7, INPUT);
 digitalWrite(3, HIGH);
 pinMode(3, OUTPUT);
 digitalWrite(3, HIGH);*/
 //*****

 // RTC initialization
 rtc.begin(); // inititalizes real time clock
 rtc.setTime(hours, minutes, seconds); // initializes rtc time
 rtc.setDate(day, month, year); // inits rtc date

 // RTC alarm setting on every 15 s resulting in 1 minute sleep period
 rtc.setAlarmSeconds(15); // sets alarm to go off in 15 seconds?
 rtc.enableAlarm(rtc.MATCH_SS); // sets alarm to go off every minute?
 rtc.attachInterrupt(alarmMatch); // attatches alarm interrupt to alarmMatch
 
 digitalWrite(LED_BUILTIN, LOW); // pulls LED pin low (on RocketScream this is pin 13?)
 USBDevice.detach(); // detaches USB interface
 rtc.standbyMode();  // rtc goes into standby mode 
}

void loop(){
 // Initialize USB and attach to host (not required if not in use)
 USBDevice.init(); // initializes USB interface
 USBDevice.attach(); // attaches USB interface
 
 digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // pulls LED pin opposite of what LED pin reads?

 // Detach USB from host (not required if not in use)
 USBDevice.detach();
 
 // Sleep until next alarm match
 rtc.standbyMode();
}

// Don't put anything in alarmMatch, unless to increment/reset counters
void alarmMatch(){

}

// Turns off all peripherals for sleep mode
void sleepReset(){
 for (pinNumber = 0; pinNumber < 23; pinNumber++){
  pinMode(pinNumber, INPUT_PULLUP);
 }

 for (pinNumber = 32; pinNumber < 42; pinNumber++){
  pinMode(pinNumber, INPUT_PULLUP);
 }

 pinMode(25, INPUT_PULLUP);
 pinMode(26, INPUT_PULLUP);

 //if (!radio.init()){}
 //radio.sleep();
 
 SerialFlash.begin(flashChipSelect);
 SerialFlash.sleep();

 pinMode(LED_BUILTIN, OUTPUT);
 
 digitalWrite(LED_BUILTIN, HIGH);
}
