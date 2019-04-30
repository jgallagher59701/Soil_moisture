
#include <Wire.h>
#include <RTClibExtended.h>

#define INT0_PIN 2    //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW
#define CLOCK_STATUS 9

#define OPERATIONAL_STATUS 8  // Feedback while testing - is it working

// Tx should not use the Serial interface except for debugging
#define DEBUG 1
#if DEBUG
#define IO(x) x
#else
#define IO(x)
#endif

RTC_DS3231 RTC;      // we are using the DS3231 RTC

// cycles the led (2Hz) to indicate that we are no more in sleep mode
void blink_times(int pin, int freq, int times) 
{
  int count = 0;
  int ledStatus = 0;
  uint16_t duration = 1000/freq;
  
  while ((times == 0) ? 1: count++ < times) {
    digitalWrite(pin, LOW);
    delay(duration);
    digitalWrite(pin, HIGH);
    delay(duration);
  }
}

char date_time_str[18];

char *iso8601_date_time(DateTime t) 
{
  char val[3];
  date_time_str[0] = '\0';
  strncat(date_time_str, itoa(t.year(), val, 10), sizeof(date_time_str));
  strncat(date_time_str, "-", sizeof(date_time_str));
  strncat(date_time_str, itoa(t.month(), val, 10), sizeof(date_time_str));
  strncat(date_time_str, "-", sizeof(date_time_str));
  strncat(date_time_str, itoa(t.day(), val, 10), sizeof(date_time_str));
  strncat(date_time_str, "T", sizeof(date_time_str));
  strncat(date_time_str, itoa(t.hour(), val, 10), sizeof(date_time_str));
  strncat(date_time_str, ":", sizeof(date_time_str));
  strncat(date_time_str, itoa(t.minute(), val, 10), sizeof(date_time_str));
  strncat(date_time_str, ":", sizeof(date_time_str));
  strncat(date_time_str, itoa(t.second(), val, 10), sizeof(date_time_str));

  return date_time_str;
 
#if 0
  s = itoa(t.year(), date_time_str, 10)
  String tm = String(t.year()) + String('-') + String(t.month()) + String('-') + String(t.day());
  tm += String('T');
  tm += String(t.hour()) + String(':') + String(t.minute()) + String(':') + String(t.second());

  return tm;
#endif
}

#ifdef DEBUG
void show_date_time(const char *msg) {
  Serial.print(msg);
  Serial.println(iso8601_date_time(RTC.now()));
  
  Serial.print("Temp: ");
  Serial.println(RTC.getTemp());

  // Add a slight delay so that the serial interface can print the message 
  // before the MCU sleeps.
  delay(100);
}
#endif

void DS3231_setup()
{  
  if (!RTC.begin()) {
    IO(Serial.println("Couldn't find RTC"));
    blink_times(CLOCK_STATUS, 10 /* Hz */, 0); // 10 Hz blink
  }

  if (RTC.lostPower()) {
    IO(Serial.println("RTC lost power, lets set the time!"));
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    blink_times(CLOCK_STATUS, 2 /* Hz */, 50); // 2 Hz blink, 50 times
  }

  digitalWrite(CLOCK_STATUS, HIGH);
  delay(1000);

  //clear any pending alarms
  RTC.armAlarm(1, false);
  RTC.clearAlarm(1);
  RTC.alarmInterrupt(1, false);
  RTC.armAlarm(2, false);
  RTC.clearAlarm(2);
  RTC.alarmInterrupt(2, false);

  // Set SQW pin to OFF (in my case it was set by default to 1Hz)
  RTC.writeSqwPinMode(DS3231_OFF);

  // Check the 32kHz osc.
  if (RTC.getEN32kHz()) {
    IO(Serial.println("32kHz osc running"));
    RTC.clearEN32kHz();
    if (RTC.getEN32kHz()) {
      IO(Serial.println("32kHz osc still running... FAIL"));
      blink_times(CLOCK_STATUS, 2 /* Hz */, 0); // 2 Hz blink forever
    }
  }

  IO(Serial.println("32kHz osc not running"));
  
  // RTC setup success, status on.
  digitalWrite(CLOCK_STATUS, HIGH);  
}

void setup() {
  // put your setup code here, to run once:
  pinMode(CLOCK_STATUS, OUTPUT);
  //Set pin D2 as INPUT for accepting the interrupt signal from DS3231
  pinMode(INT0_PIN, INPUT);

  IO(while (!Serial));

  IO(Serial.begin(9600));
  IO(delay(100));

  // The DS3231 requires the Wire library
  Wire.begin();
  DS3231_setup();

  delay(10 * 1000);
  
  // Add a delay here to see the status info and then turn it off when not in DEBUG mode
#if !DEBUG
  digitalWrite(CLOCK_STATUS, LOW);
#endif
}

void loop() {
  // put your main code here, to run repeatedly:
 IO(show_date_time("Time: "));

 blink_times(OPERATIONAL_STATUS, 1, 5);

 blink_times(OPERATIONAL_STATUS, 2, 10);

 blink_times(OPERATIONAL_STATUS, 10, 50);
}
