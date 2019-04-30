
// -*- mode: C++ -*-
//
// Soil Moisture sensor
//
// Transmit information to a second LoRa radio unit. v 0.1
// Add the RTC and sleep mode v 0.2

#include <SPI.h>
#include <Wire.h>
#include <LowPower.h>

#include <RH_RF95.h>
#include <RTClibExtended.h>

#define RFM95_INT 3
#define RFM95_RST 6
#define RFM95_EN 7
#define RFM95_CS 10

#define LORA_INIT_STATUS 4

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

#define INT0 2    //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW
#define CLOCK_STATUS 9

#define OPERATIONAL_STATUS 8

// Tx should not use the Serial interface except for debugging
#define DEBUG 0
#if DEBUG
#define IO(x) x
#else
#define IO(x)
#endif

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

RTC_DS3231 RTC;      // we are using the DS3231 RTC

// From https://arduino.stackexchange.com/questions/23526/measure-different-vcc-using-1-1v-bandgap
// To get Internal 1.1V reference: ADMUX = bit (REFS0) | bit (REFS1);
// Vcc     Aref
// ------------
// 5.00   1.085
// 4.00   1.081
// 3.30   1.080
// 3.00   1.079

#define AREF_VOLTAGE_X1000 1080L

// Configure the radio. If an error is detected, blink the status LED.
// If an error is detected, this function does not return.
void lora_setup() 
{
  // LED to show the LORA radio has been configured.
  digitalWrite(LORA_INIT_STATUS, LOW);
  
  // LORA manual reset
  digitalWrite(RFM95_RST, HIGH);
  
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Defaults for RFM95 after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5,
  // Sf = 128chips/symbol, CRC on. If the init fails, LORA status LED blinks at
  // 10Hz
  while (!rf95.init()) {
    IO(Serial.println("LoRa radio init failed"));
    blink_times(LORA_INIT_STATUS, 10 /* Hz */, 0); // 10 Hz blink
  }
  IO(Serial.println("LoRa radio init OK!"));

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM.
  // If frequenct set fails, blink at 2Hz forever.
  if (!rf95.setFrequency(RF95_FREQ)) {
    IO(Serial.println("setFrequency failed"));
    blink_times(LORA_INIT_STATUS, 2, 0); // 2 Hz blink
  }
  IO(Serial.print("Set Freq to: ")); 
  IO(Serial.println(RF95_FREQ));

  // None of these return error codes.
  // Setup Spreading Factor (chips/symbol) (n = 6 ~ 12, where Sf=2^n (eg 6 --> 2^6 == 64 chips/sym)
  rf95.setSpreadingFactor(7);
  
  // Setup BandWidth, option: 7800,10400,15600,20800,31200,41700,62500,125000,250000,500000
  // Higher == higher data rates
  rf95.setSignalBandwidth(125000);
  
  // Setup Coding Rate:5(4/5),6(4/6),7(4/7),8(4/8) (higher == better error correction)
  rf95.setCodingRate4(5);
 
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  // LORA setup success, status on.
  digitalWrite(LORA_INIT_STATUS, HIGH);  
}

void DS3231_setup()
{  
  if (!RTC.begin()) {
    IO(Serial.println("Couldn't find RTC"));
    blink_times(LORA_INIT_STATUS, 10 /* Hz */, 0); // 10 Hz blink
  }

  if (RTC.lostPower()) {
    IO(Serial.println("RTC lost power, lets set the time!"));
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    blink_times(LORA_INIT_STATUS, 2 /* Hz */, 50); // 2 Hz blink, 50 times
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
      blink_times(LORA_INIT_STATUS, 2 /* Hz */, 0); // 2 Hz blink forever
    }
  }

  IO(Serial.println("32kHz osc not running"));
  
  // RTC setup success, status on.
  digitalWrite(CLOCK_STATUS, HIGH);  
}

uint16_t packetnum = 0;   // packet counter, we increment per xmission
int16_t rssi = 0;         // rssi and snr are read for the ACK received in response to a Tx
int snr = 0;

// Send basic information about the sensor plus the information in \arg data.
// Wait for a response and record the SNR and RSSI of the response (used in the
// next send_packet() transmission.
void send_packet(String time_stamp, String data)
{
  IO(Serial.println("Sending to rf95_server"));
  
  // Send a message to rf95_server  
  // The RSSI and SNR are for the most recent ACK received from the server in response to 
  // the client's message. The message is 'Hello World#<packetnum>,<Vcc>,<rssi>,<snr>'. 
  // Vcc is X100 (seegetBandgap()).
  String message = time_stamp + String(',');
  message += String(++packetnum) + String(',');
  message += String(get_bandgap())+ String(',');
  message += String(rssi) + String(',');
  message += String(snr) + String(',');
  message += data;
 
  rf95.send((uint8_t *)message.c_str(), message.length() + 1);

  // Now wait for a reply
  IO(Serial.println("Waiting for packet to complete..."));
  
  rf95.waitPacketSent();
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  IO(Serial.println("Waiting for reply..."));
  
  if (rf95.waitAvailableTimeout(1000)) { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len)) {
      // Update rssi and snr
      rssi = rf95.lastRssi();
      snr = rf95.lastSNR();
      
      IO(Serial.print("Got reply: "));
      IO(Serial.println((char*)buf));
      IO(Serial.print("RSSI: "));
      IO(Serial.println(rssi, DEC));    
    }
    else {
      rssi = 0;
      snr = 0;
      
      IO(Serial.println("Receive failed"));
    }
  }
  else {
    IO(Serial.println("No reply, is there a listener around?"));
  }
}

// Function created to obtain chip's actual Vcc voltage value, using internal bandgap reference
// This demonstrates ability to read processors Vcc voltage and the ability to maintain A/D calibration with changing Vcc
// Now works for 168/328 and mega boards.
// Thanks to "Coding Badly" for direct register control for A/D mux
// 1/9/10 "retrolefty"

uint16_t get_bandgap()   // Returns actual value of Vcc (x 100)
{    
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
     // For mega boards
     const long InternalReferenceVoltage = 1115L;  // Adjust this value to your boards specific internal BG voltage x1000
        // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc reference
        // MUX4 MUX3 MUX2 MUX1 MUX0  --> 11110 1.1V (VBG)         -Selects channel 30, bandgap voltage, to measure
     ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR)| (0<<MUX5) | (1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
  
#else
     // For 168/328 boards
     const long InternalReferenceVoltage = AREF_VOLTAGE_X1000;  // Adjust this value to your boards specific internal BG voltage x1000
        // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
        // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
     ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
       
#endif
     delay(50);  // Let mux settle a little to get a more stable A/D conversion
        // Start a conversion  
     ADCSRA |= _BV( ADSC );
        // Wait for it to complete
     while( ( (ADCSRA & (1<<ADSC)) != 0 ) );
        // Scale the value
     uint16_t results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L; // calculates for straight line value 
     return results; 
}

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

#ifdef DEBUG
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
#endif

// Set up the radio, halt here if init fails; LEDS provide radio status.
// If one LED lights, that indicates a failure. If both light, that indicates
// Success
void setup() 
{
  pinMode(LORA_INIT_STATUS, OUTPUT);
  pinMode(RFM95_EN, OUTPUT);
  pinMode(CLOCK_STATUS, OUTPUT);
  //Set pin D2 as INPUT for accepting the interrupt signal from DS3231
  pinMode(INT0, INPUT);
   
  IO(while (!Serial));

  IO(Serial.begin(9600));
  IO(delay(100));

  // The DS3231 requires the Wire library
  Wire.begin();
  DS3231_setup();

  // Power on the lora
  digitalWrite(RFM95_EN, HIGH);

  lora_setup();

  delay(10 * 1000);
  
  // Add a delay here to see the status info and then turn it off when not in DEBUG mode
#if !DEBUG
  digitalWrite(LORA_INIT_STATUS, LOW);
  digitalWrite(CLOCK_STATUS, LOW);
#endif
}

byte AlarmFlag = 0;

// Callback for interrupt
void wake_up()
{
  AlarmFlag = 1;
}

void loop()
{ 
   //On first loop we enter the sleep mode
  if (AlarmFlag == 0) {
    IO(show_date_time("Setting the alarm at: "));
    delay(10);

    // Power down the lora
    digitalWrite(RFM95_EN, LOW);
    
    // Set alarm every minute at the 15 second mark
    RTC.setAlarm(ALM1_MATCH_SECONDS, 15, 0, 0, 0);
    RTC.alarmInterrupt(1, true);

    // wake_up() sets AlarmFlag
    attachInterrupt(0, wake_up, LOW);                       //use interrupt 0 (pin 2) and run function wake_up when pin 2 gets LOW 
    digitalWrite(OPERATIONAL_STATUS, LOW);                  //switch-off the led for indicating that we enter the sleep mode

    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);   //arduino enters sleep mode here
    detachInterrupt(0);                                    //execution resumes from here after wake-up

    //When exiting the sleep mode we clear the alarm
    RTC.armAlarm(1, false);
    RTC.clearAlarm(1);
    RTC.alarmInterrupt(1, false);

    // Power up the lora
    digitalWrite(RFM95_EN, HIGH);
    lora_setup();
    digitalWrite(LORA_INIT_STATUS, LOW);
  }

  send_packet(iso8601_date_time(RTC.now()), String(RTC.getTemp()));

  IO(show_date_time("Woke up at: "));
  
  blink_times(OPERATIONAL_STATUS, 1, 10);
 
  AlarmFlag = 0;
}
