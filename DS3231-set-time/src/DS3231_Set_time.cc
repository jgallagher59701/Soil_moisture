
/**
 * (Re)set the DS3231 clock to now. 
 * 
 * To use this, clean the build dir, then build and upload in one
 * operation. 
 */

#include <Arduino.h>

#include <Wire.h>
#include <RTClibExtended.h>

// pin for the LED to blink indicating various clock status 'codes.'
#define STATUS 13

// Blink three times to indicate clock status
#define CLOCK_STATUS 3
#define ERROR_OCCURRED 0

// Number opf seconds it takes to compile, upload and run this program
// at 9600 baud. Used to adjust the clock
#define TIME_OFFSET 12

// If ADJUST_TIME is 1, set the clock using the values from the time the
// program was compiled. If 0, just read the DS3231 chip's time.
#define ADJUST_TIME 0

// Real time clock
RTC_DS3231 RTC;      // we are using the DS3231 RTC

// It would be better to use the blink.cpp/h code in src.

// Blink N times, 1/4s on. Repeat M times. If M is 0, repeat forever.
void blink_times(int pin, int N, int M) {

    const int one_second = 1000;
    const int quarter_second = 250;

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
    strncat(date_time_str, itoa(t.year(), val, 10), sizeof(date_time_str) - 1);
    strncat(date_time_str, "-", sizeof(date_time_str) - 1);
    if (t.month() < 10) strncat(date_time_str, "0", sizeof(date_time_str) - 1);
    strncat(date_time_str, itoa(t.month(), val, 10), sizeof(date_time_str) - 1);
    strncat(date_time_str, "-", sizeof(date_time_str) - 1);
    if (t.day() < 10) strncat(date_time_str, "0", sizeof(date_time_str) - 1);
    strncat(date_time_str, itoa(t.day(), val, 10), sizeof(date_time_str) - 1);
    strncat(date_time_str, "T", sizeof(date_time_str) - 1);
    if (t.hour() < 10) strncat(date_time_str, "0", sizeof(date_time_str) - 1);
    strncat(date_time_str, itoa(t.hour(), val, 10), sizeof(date_time_str) - 1);
    strncat(date_time_str, ":", sizeof(date_time_str) - 1);
    if (t.minute() < 10) strncat(date_time_str, "0", sizeof(date_time_str) - 1);
    strncat(date_time_str, itoa(t.minute(), val, 10), sizeof(date_time_str) - 1);
    strncat(date_time_str, ":", sizeof(date_time_str) - 1);
    if (t.second() < 10) strncat(date_time_str, "0", sizeof(date_time_str) - 1);
    strncat(date_time_str, itoa(t.second(), val, 10), sizeof(date_time_str) - 1);

    return date_time_str;
}

void setup() {
    // The DS3231 requires the Wire library
    Wire.begin();

    if (!RTC.begin()) {
        blink_times(STATUS, CLOCK_STATUS, ERROR_OCCURRED);
    }

#if ADJUST_TIME
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));

    DateTime adjusted_time(RTC.now() + TIME_OFFSET);
    RTC.adjust(adjusted_time);
#endif\
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));  // Start the serial port

    while (!Serial);
    Serial.begin(9600);

    Serial.print(F("Set the time to: "));
    Serial.print(F(__DATE__));
    Serial.print(F(", "));
    Serial.println(F(__TIME__));

    Serial.print(F("Current time: "));
    Serial.println(iso8601_date_time(RTC.now()));
}


void loop() {
    delay(5000);

    Serial.print(F("Current time: "));
    Serial.println(iso8601_date_time(RTC.now()));
}
