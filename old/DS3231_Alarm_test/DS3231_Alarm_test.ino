/**
 *  Sets an alarm using a DS3231 device, goes to sleep and wakes up again
 * @author: dev-eth0
 * @url: https://www.dev-eth0.de/
 */

#include <DS3231.h> // https://github.com/NorthernWidget/DS3231
#include <Wire.h>
#include <LowPower.h> // https://github.com/rocketscream/Low-Power

DS3231 Clock;

// Some static test-date for the RTC
byte Year = 2017;
byte Month = 9;
byte Date = 17;
byte Hour = 19;
byte Minute = 29;
byte Second = 30;

// Interrupt Pin used
static const byte wakeUpPin = 2;


// Those are the ALARM Bits that can be used
// They need to be combined into a single value (see below)
// Found here: https://github.com/mlepard/ArduinoChicken/blob/master/roboCoop/alarmControl.ino
#define ALRM1_MATCH_EVERY_SEC  0b1111  // once a second
#define ALRM1_MATCH_SEC        0b1110  // when seconds match
#define ALRM1_MATCH_MIN_SEC    0b1100  // when minutes and seconds match
#define ALRM1_MATCH_HR_MIN_SEC 0b1000  // when hours, minutes, and seconds match

#define ALRM2_ONCE_PER_MIN     0b111   // once per minute (00 seconds of every minute)
#define ALRM2_MATCH_MIN        0b110   // when minutes match
#define ALRM2_MATCH_HR_MIN     0b100   // when hours and minutes match

int ledState = HIGH;

void setup() {
  // Start the serial port
  Serial.begin(9600);
  Serial.println("Alarm Test");

  digitalWrite(LED_BUILTIN, ledState);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Configure Interrupt Pin
  pinMode(wakeUpPin, INPUT_PULLUP);
  digitalWrite(wakeUpPin, HIGH);

  // Start the I2C interface
  Wire.begin();

  // Set time
  Clock.setClockMode(false);
  
#if 0
  // FIXME. Assume the clock time is set.
  Clock.setYear(Year);
  Clock.setMonth(Month);
  Clock.setDate(Date);
  Clock.setHour(Hour);
  Clock.setMinute(Minute);
  Clock.setSecond(Second);
#endif

  // Set alarm
  Serial.println("Setting alarm");

  // NB: Modified to only use Alarm #1
  
  // This is the interesting part which sets the AlarmBits and configures, when the Alarm be triggered
  byte ALRM1_SET = ALRM1_MATCH_SEC; // trigger A1 whenever the seconds match
  // ALRM1_MATCH_MIN_SEC; // trigger A1 when minute and second match
  byte ALRM2_SET = ALRM2_MATCH_MIN;     // trigger A2 when minute matches (and second is 0 as A2 does not support seconds)

  // combine the AlarmBits
  int ALARM_BITS = ALRM2_SET;
  ALARM_BITS <<= 4;
  ALARM_BITS |= ALRM1_SET;
  
  // Trigger Alarm when Minute == 30 or 0
  // Clock.setA1Time(Day, Hour, Minute, Second, AlarmBits, DayOfWeek, 12 hour mode, PM)
  Clock.setA1Time(0, 0, 0, 0, ALARM_BITS, false, false, false); 
  // Clock.setA2Time(Day, Hour, Minute, AlarmBits, DayOfWeek, 12 hour mode, PM)
  Clock.setA2Time(0, 0, 30, ALARM_BITS, false, false, false);    

  // Turn on Alarm
  Clock.turnOnAlarm(1);
  Clock.turnOnAlarm(2);
  
  Serial.println(ALARM_BITS, BIN);
  Serial.print("Alarm 1: ");
  Serial.println(Clock.checkAlarmEnabled(1));
  Serial.print("Alarm 2: ");
  Serial.println(Clock.checkAlarmEnabled(2));
  
  DateTime now = RTClib::now();
  
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();


  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, FALLING);

  // sleep
  delay(500);

  DateTime now = RTClib::now();
  
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

// loop is started once the device wakes up again
void loop() {
    Serial.print("Alarm 1: ");
  Serial.println(Clock.checkAlarmEnabled(1));

  blinkLED();
  delay(1000);
}

void blinkLED() {
  #if 0
  if (ledState == LOW) {
    ledState = HIGH;
  } else {
    ledState = LOW;
  }
  digitalWrite(LED_BUILTIN, ledState);
  #endif
  digitalWrite(LED_BUILTIN, LOW);
}

void wakeUp() {
  // wake up again
  Serial.println("Woke up this morning...");
}
