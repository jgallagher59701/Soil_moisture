
/**
 * (Re)set the DS3231 clock to now.
 */

#include <Wire.h>
#include <RTClibExtended.h>

#define STATUS 13

#define CLOCK_STATUS 3

#define START 1
#define COMPLETED 1
#define ERROR_OCCURRED 0

#define FORCE_CLOCK_RESET 1

// Real time clock
RTC_DS3231 RTC;      // we are using the DS3231 RTC

// Blink N times, 1/4s on. Repeat M times. If M is 0, repeat forever.
void blink_times(int pin, int N, int M) {
  
  #define one_second 1000
  #define quarter_second 250
  
  int count = 0;
  while ((M == 0) ? 1 : count < M) {
    count++;
    for (int i = 0; i < N; ++i) {
      digitalWrite(pin, HIGH);
      delay(quarter_second);
      digitalWrite(pin, LOW);
      delay(quarter_second);
    }
    delay(one_second);
  }
}

// Given a DateTime instance, return a pointer to static string that holds
// an ISO 8601 print representation of the object.

char *iso8601_date_time(DateTime t) {
  static char date_time_str[32];
  
  char val[12];
  date_time_str[0] = '\0';
  strncat(date_time_str, itoa(t.year(), val, 10), sizeof(date_time_str)-1);
  strncat(date_time_str, "-", sizeof(date_time_str)-1);
  if (t.month() < 10) strncat(date_time_str, "0", sizeof(date_time_str)-1);
  strncat(date_time_str, itoa(t.month(), val, 10), sizeof(date_time_str)-1);
  strncat(date_time_str, "-", sizeof(date_time_str)-1);
  if (t.day() < 10) strncat(date_time_str, "0", sizeof(date_time_str)-1);
  strncat(date_time_str, itoa(t.day(), val, 10), sizeof(date_time_str)-1);
  strncat(date_time_str, "T", sizeof(date_time_str)-1);
  if (t.hour() < 10) strncat(date_time_str, "0", sizeof(date_time_str)-1);
  strncat(date_time_str, itoa(t.hour(), val, 10), sizeof(date_time_str)-1);
  strncat(date_time_str, ":", sizeof(date_time_str)-1);
  if (t.minute() < 10) strncat(date_time_str, "0", sizeof(date_time_str)-1);
  strncat(date_time_str, itoa(t.minute(), val, 10), sizeof(date_time_str)-1);
  strncat(date_time_str, ":", sizeof(date_time_str)-1);
  if (t.second() < 10) strncat(date_time_str, "0", sizeof(date_time_str)-1);
  strncat(date_time_str, itoa(t.second(), val, 10), sizeof(date_time_str)-1);

  return date_time_str;
}

void setup() {
  // The DS3231 requires the Wire library
  Wire.begin();
  
  if (!RTC.begin()) {
    blink_times(STATUS, CLOCK_STATUS, ERROR_OCCURRED);
  }

  RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));  // Start the serial port
  
  while (!Serial) ;
  Serial.begin(9600);
  
  Serial.print(F("Set the time to: "));
  Serial.print(F(__DATE__));
  Serial.print(F(", "));
  Serial.println(F(__TIME__));

  Serial.print(F("Current time: "));
  Serial.println(iso8601_date_time(RTC.now()));
}


void loop() {
  
}
