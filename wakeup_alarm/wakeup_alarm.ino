#include <Wire.h>
#include <RTClibExtended.h>
#include <LowPower.h>

#define wakePin 2    //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW
#define ledPin 13    //use arduino on-board led for indicating sleep or wakeup status

RTC_DS3231 RTC;      //we are using the DS3231 RTC

byte AlarmFlag = 0;
byte ledStatus = 1;

void setup() {
  Serial.begin(9600);
  
  while (!Serial);  // Wait for the monitor to open

  Wire.begin();
  
  if (!RTC.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (RTC.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  //Set pin D2 as INPUT for accepting the interrupt signal from DS3231
  pinMode(wakePin, INPUT);

  //switch-on the on-board led for 1 second for indicating that the sketch is ok and running
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(1000);

  //clear any pending alarms
  RTC.armAlarm(1, false);
  RTC.clearAlarm(1);
  RTC.alarmInterrupt(1, false);
  RTC.armAlarm(2, false);
  RTC.clearAlarm(2);
  RTC.alarmInterrupt(2, false);

  //Set SQW pin to OFF (in my case it was set by default to 1Hz)
  //The output of the DS3231 INT pin is connected to this pin
  //It must be connected to arduino D2 pin for wake-up
  RTC.writeSqwPinMode(DS3231_OFF);

  // Set alarm1 every day at 18:33
  // RTC.setAlarm(ALM1_MATCH_HOURS, 59, 18, 0);   //set your wake-up time here

  // Check the 32kHz osc.
  RTC.setEN32kHz();
  
  if (RTC.getEN32kHz()) {
    Serial.println("32kHz osc running");
    RTC.clearEN32kHz();
    if (RTC.getEN32kHz()) {
      Serial.println("32kHz osc still running... FAIL");
      while (1);
    }
  }

  Serial.println("32kHz osc not running");
}

void wakeUp()        // here the interrupt is handled after wakeup
{
}

#define BLINKS 20
int count = 0;  // Loop for BLINKS blinks of the LED and then set teh alarm again

void loop() {

  //On first loop we enter the sleep mode
  if (AlarmFlag == 0) {
    show_date_time("Setting the alarm at: ");

    // Set alarm every minute at the 15 second mark
    RTC.setAlarm(ALM1_MATCH_SECONDS, 15, 0, 0, 0);
    RTC.alarmInterrupt(1, true);

    attachInterrupt(0, wakeUp, LOW);                       //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW 
    digitalWrite(ledPin, LOW);                             //switch-off the led for indicating that we enter the sleep mode
    ledStatus = 0;                                         //set the led status accordingly
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);   //arduino enters sleep mode here
    detachInterrupt(0);                                    //execution resumes from here after wake-up

    //When exiting the sleep mode we clear the alarm
    RTC.armAlarm(1, false);
    RTC.clearAlarm(1);
    RTC.alarmInterrupt(1, false);
    AlarmFlag = 1;
  }

  show_date_time("Woke up at: ");
  blink_led(ledPin, BLINKS);
 
  AlarmFlag = 0;
}

// cycles the led (2Hz) to indicate that we are no more in sleep mode
void blink_led(int pin, int times) {
  int count = 0;
  int ledStatus = 0;
  
  while (count++ < times) {
    if (ledStatus == 0) {
      ledStatus = 1;
      digitalWrite(ledPin, HIGH);
    }
    else {
      ledStatus = 0;
      digitalWrite(ledPin, LOW);
    }
  
    delay(500);
  }
}

// TODO Use a char array of fixed size, drop the String class and add leading zeros.
// zero (0) is 48 in ASCII.

String iso8601_date_time(DateTime t) {
#if 0
 char radiopacket[20] = "Hello World #      ";
  itoa(packetnum++, radiopacket+13, 10);
  radiopacket[19] = 0;

  char val[2];
  if (t.day() < 10) {
    val[0] = 48;
    val[1] = 48 + t.day()
  }
  else {
    val[0] = 48 + t.day() / 10;
    val[1] = 48 + t.day() % 10;
  }
#endif
  String tm = String(t.year()) + String('-') + String(t.month()) + String('-') + String(t.day());
  tm += String('T');
  tm += String(t.hour()) + String(':') + String(t.minute()) + String(':') + String(t.second());

  return tm;
}

void show_date_time(String msg) {
  Serial.print(msg);
  Serial.println(iso8601_date_time(RTC.now()));
  
  Serial.print("Temp: ");
  float temp = RTC.getTemp();
  Serial.println(temp);

  // Add a slight delay so that the serial interface can print the message 
  // before the MCU sleeps.
  delay(100);
}
