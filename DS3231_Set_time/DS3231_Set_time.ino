
/**
 * (Re)set the DS3231 clock to now.
 */

#include <Wire.h>
#include <RTClibExtended.h>

#define INT0_PIN 2
#define CLOCK_POWER 9
#define CLOCK_STATUS 13

#define FORCE_CLOCK_RESET 1

// Tx should not use the Serial interface except for debugging
#define DEBUG 1
#if DEBUG
#define IO(x) do { x; } while (0)
#else
#define IO(x)
#endif

// Real time clock
RTC_DS3231 RTC;      // we are using the DS3231 RTC

void setup() {
  // put your setup code here, to run once:
 // Turn off unused features; init code will turn on things that are used
  // shutdown_unused();
  
  // Configure Interrupt Pin
  //pinMode(INT0_PIN, INPUT_PULLUP);
  pinMode(INT0_PIN, INPUT);
  pinMode(CLOCK_STATUS, INPUT);
  pinMode(CLOCK_POWER, OUTPUT);
  
  digitalWrite(INT0_PIN, HIGH);
  
#if DEBUG 
  // Start the serial port
  while (!Serial) ;
  Serial.begin(9600);
  Serial.println(F("Set the time: "));
#endif

  digitalWrite(CLOCK_POWER, HIGH);
  
  // Configures the clock and sets the status HIGH
  DS3231_setup();
}

// cycles the led at 'pin' ('freq' Hz) for t seconds to indicate various things
// If 't' is zero, blink forever (i.e., never return because an error was
// detected.
void blink_times(int pin, int freq, int t) {
  int count = 0;
  uint16_t duration = 1000 / (freq * 2);
  uint16_t times = t * freq;

  while ((times == 0) ? 1 : count++ < times) {
    digitalWrite(pin, LOW);
    delay(duration);
    digitalWrite(pin, HIGH);
    delay(duration);
  }
}

void DS3231_setup() 
{
  // The DS3231 requires the Wire library
  Wire.begin();
  
  if (!RTC.begin()) {
    IO(Serial.println(F("Couldn't find RTC")));
    blink_times(CLOCK_STATUS, 10 /* Hz */, 0); // 10 Hz blink forever
  }

  if (RTC.lostPower() || FORCE_CLOCK_RESET) {
    IO(Serial.println(F("RTC lost power, lets set the time!")));
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    blink_times(CLOCK_STATUS, 2 /* Hz */, 5); // 2 Hz blink, 10 times
  }

  // RTC setup success, status on.
  IO(Serial.println(F("RTC init OK!")));
}

void loop() {
  // put your main code here, to run repeatedly:

  blink_times(CLOCK_STATUS, 2 /* Hz */, 0);
}
