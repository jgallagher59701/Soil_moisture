
// -*- mode: C++ -*-
//
// Soil Moisture sensor
//
// Transmit information to a second LoRa radio unit. v 0.1
// Add the RTC and sleep mode v 0.2
// Add SD card logging v 0.3

#include <SPI.h>
#include <Wire.h>
#include <LowPower.h>

#define USE_SD 1

#if USE_SD
#include <SD.h>
#endif

#include <RH_RF95.h>
#include <RTClibExtended.h>

#define RFM95_INT 3
#define RFM95_RST 6
#define RFM95_EN 7
#define RFM95_CS 5
// 10

#define LORA_INIT_STATUS 4

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

#define INT0_PIN 2    //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW
#define CLOCK_STATUS 9

#define SD_CARD_CS 10
// 5

#define OPERATIONAL_STATUS 8  // Feedback while testing - is it working?

#define SLEEP 1

// Tx should not use the Serial interface except for debugging
#define DEBUG 0
#if DEBUG
#define IO(x) do { x; } while (0)
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
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Defaults for RFM95 after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5,
  // Sf = 128chips/symbol, CRC on. If the init fails, LORA status LED blinks at
  // 10Hz
  while (!rf95.init()) {
    IO(Serial.println(F("LoRa radio init failed")));
    blink_times(LORA_INIT_STATUS, 10 /* Hz */, 0); // 10 Hz blink
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM.
  // If frequenct set fails, blink at 2Hz forever.
  if (!rf95.setFrequency(RF95_FREQ)) {
    IO(Serial.println(F("setFrequency failed")));
    blink_times(LORA_INIT_STATUS, 2, 0); // 2 Hz blink
  }
  IO(Serial.print(F("Set Freq to: "))); 
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
  IO(Serial.println(F("LoRa radio init OK!")));
  digitalWrite(LORA_INIT_STATUS, HIGH);  
}

void DS3231_setup()
{  
  // The DS3231 requires the Wire library
  Wire.begin();
  if (!RTC.begin()) {
    IO(Serial.println(F("Couldn't find RTC")));
    blink_times(CLOCK_STATUS, 10 /* Hz */, 0); // 10 Hz blink
  }

  if (RTC.lostPower()) {
    IO(Serial.println(F("RTC lost power, lets set the time!")));
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    blink_times(CLOCK_STATUS, 2 /* Hz */, 50); // 2 Hz blink, 50 times
  }

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
    IO(Serial.println(F("32kHz osc running")));
    RTC.clearEN32kHz();
    if (RTC.getEN32kHz()) {
      IO(Serial.println(F("32kHz osc still running... FAIL")));
      blink_times(CLOCK_STATUS, 2 /* Hz */, 0); // 2 Hz blink forever
    }
  }

  IO(Serial.println(F("32kHz osc not running")));
  
  // RTC setup success, status on.
  IO(Serial.println(F("RTC init OK!")));
  digitalWrite(CLOCK_STATUS, HIGH);  
}

// Initialize the SD card for dat logging. If there's no card, blink the 
// operational status LED and continue on.
// TODO Should it be a fatal error to not have a SD card?
#if USE_SD
void sd_card_setup()
{
  // Configure the SD card I/O
  if (!SD.begin(SD_CARD_CS)) {
    IO(Serial.println(F("Card failed, or not present")));
    blink_times(OPERATIONAL_STATUS, 10 /* Hz */, 100); // 10 Hz blink
  }

  IO(Serial.println(F("card initialized.")));
}
#endif

uint16_t packetnum = 0;   // packet counter, we increment per xmission
int16_t rssi = 0;         // rssi and snr are read for the ACK received in response to a Tx
int snr = 0;

// Send basic information about the sensor plus the information in \arg data.
// Wait for a response and record the SNR and RSSI of the response (used in the
// next send_packet() transmission.
// 
// Return a pointer to the packet sent.

char packet[64];

char *send_packet(char *time_stamp, float temp)
{
  IO(Serial.println(F("Sending to rf95_server")));
  
  // Send a message to rf95_server  
  // The RSSI and SNR are for the most recent ACK received from the server in response to 
  // the client's message. The message is 'Hello World#<packetnum>,<Vcc>,<rssi>,<snr>,<temp>'. 
  // Vcc is X100 (see get_bandgap()) and temp is X100 as well.
  char val[8];
  packet[0] = '\0';
  
  strncpy(packet, time_stamp, sizeof(packet));  // 17 
  strncat(packet, ",", sizeof(packet));
  
  strncat(packet, itoa(++packetnum, val, 10), sizeof(packet)); // 5
  strncat(packet, ",", sizeof(packet));

  strncat(packet, itoa(get_bandgap(), val, 10), sizeof(packet)); // 3
  strncat(packet, ",", sizeof(packet));

  strncat(packet, itoa(rssi, val, 10), sizeof(packet)); // 6
  strncat(packet, ",", sizeof(packet));
  
  strncat(packet, itoa(snr, val, 10), sizeof(packet)); // 3
  strncat(packet, ",", sizeof(packet));

  strncat(packet, itoa((int)(temp*100), val, 10), sizeof(packet));
  //strncat(packet, dtostrf((double)temp, 5 /*width*/, 1/*precision*/, val), sizeof(packet));
  
  rf95.send((uint8_t *)packet, strnlen(packet, sizeof(packet)) + 1);

  // Now wait for a reply
  IO(Serial.println(F("Waiting for packet to complete...")));
  
  rf95.waitPacketSent();
  uint8_t buf[64 /*RH_RF95_MAX_MESSAGE_LEN*/];
  uint8_t len = sizeof(buf);

  IO(Serial.println(F("Waiting for reply...")));
  
  if (rf95.waitAvailableTimeout(1000)) { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len)) {
      // Update rssi and snr
      rssi = rf95.lastRssi();
      snr = rf95.lastSNR();
      
      IO(Serial.print(F("Got reply: ")));
      IO(Serial.println((char*)buf));
      IO(Serial.print(F("RSSI: ")));
      IO(Serial.println(rssi, DEC));    
    }
    else {
      rssi = 0;
      snr = 0;
      
      IO(Serial.println(F("Receive failed")));
    }
  }
  else {
    IO(Serial.println(F("No reply, is there a listener around?")));
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

char date_time_str[32];

char *iso8601_date_time(DateTime t) 
{
  char val[8];
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
}

// This function gets and releases the SPI bus so that the lora does not have too.
// \param packet Null terminated global string to write to the log file.
#if USE_SD
void log_measurement()
{
  // TODO Turn on the card circuit.

  IO(Serial.print(F("logging packet: ")));
  IO(Serial.println(packet));
  

  File f = SD.open(F("datalog.txt"), FILE_WRITE);
  if (f) {
    f.println(packet);
    f.close();
    blink_times(OPERATIONAL_STATUS, 2, 10);
  }
  else {
    IO(Serial.println(F("Could not open SD card file")));
    blink_times(OPERATIONAL_STATUS, 10, 50);
  }

  IO(Serial.flush());
  // TODO Turn off the card circuit
}
#endif

#if DEBUG
void show_date_time(const char *msg) 
{
  Serial.print(msg);
  Serial.println(iso8601_date_time(RTC.now()));
  
  Serial.print(F("Temp: "));
  Serial.println(RTC.getTemp());
  Serial.flush();
}
#else
void show_date_time(const char *) 
{
}
#endif

void lora_select()
{
  digitalWrite(RFM95_CS, LOW);
  digitalWrite(SD_CARD_CS, HIGH);
}

void sd_select()
{
  digitalWrite(SD_CARD_CS, LOW);
  digitalWrite(RFM95_CS, HIGH);
}

// Set up the radio, halt here if init fails; LEDS provide radio status.
// If one LED lights, that indicates a failure. If both light, that indicates
// Success
void setup() 
{
  pinMode(LORA_INIT_STATUS, OUTPUT);
  pinMode(RFM95_EN, OUTPUT);
  pinMode(CLOCK_STATUS, OUTPUT);
  pinMode(OPERATIONAL_STATUS, OUTPUT);
  pinMode(RFM95_CS, OUTPUT);
#if USE_SD
  pinMode(SD_CARD_CS, OUTPUT);
#endif

  pinMode(INT0_PIN, INPUT); //Set pin D2 as INPUT for accepting the interrupt signal from DS3231

#if USE_SD || DEBUG
  while (!Serial);
  Serial.begin(9600);
  delay(100);
#endif

  DS3231_setup();

  // Power on the lora
  digitalWrite(RFM95_EN, HIGH);

#if USE_SD
  lora_select();
#endif
  lora_setup();

  // The SD card functions all select it and then select the lora when done.
#if USE_SD
  sd_select();
  sd_card_setup();
#endif

  blink_times(OPERATIONAL_STATUS, 1, 5);
  
  // Add a delay here to see the status info and then turn it off when not in DEBUG mode
#if !DEBUG
  digitalWrite(LORA_INIT_STATUS, LOW);
  digitalWrite(CLOCK_STATUS, LOW);
  digitalWrite(OPERATIONAL_STATUS, LOW);
#endif
}

byte AlarmFlag = 0;

// Callback for interrupt
void wake_up()
{
}

void loop()
{
   //On first loop we enter the sleep mode
  if (AlarmFlag == 0) {
    IO(show_date_time("Setting the alarm at: "));

    // Set alarm every minute at the 15 second mark
    RTC.setAlarm(ALM1_MATCH_SECONDS, 15, 0, 0, 0);
    RTC.alarmInterrupt(1, true);

    // wake_up() sets AlarmFlag
    attachInterrupt(0, wake_up, LOW);                       //use interrupt 0 (pin 2) and run function wake_up when pin 2 gets LOW 
    digitalWrite(OPERATIONAL_STATUS, LOW);                  //switch-off the led for indicating that we enter the sleep mode

#if SLEEP
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);   //arduino enters sleep mode here
#endif
    detachInterrupt(0);                                    //execution resumes from here after wake-up

    //When exiting the sleep mode we clear the alarm
    RTC.armAlarm(1, false);
    RTC.clearAlarm(1);
    RTC.alarmInterrupt(1, false);
  }

  // send_packet(...) writes to a global, then sends the packet. log_measurement() writes
  // the same packet to the SD card.
    
  // Power up the lora
  digitalWrite(RFM95_EN, HIGH);
  lora_select();
  lora_setup();
    
  send_packet(iso8601_date_time(RTC.now()), RTC.getTemp());

  // Power down the lora
  digitalWrite(RFM95_EN, LOW);
#if !DEBUG
  digitalWrite(LORA_INIT_STATUS, LOW);
#endif

  sd_select();
  delay(100);

  log_measurement();

  IO(show_date_time("Woke up at: "));
  
  blink_times(OPERATIONAL_STATUS, 1, 5);
 
  AlarmFlag = 0;
}
