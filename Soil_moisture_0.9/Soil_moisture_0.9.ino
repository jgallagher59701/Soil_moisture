/**
 *  Sets an alarm using a DS3231 device, goes to sleep and wakes up again
 * @author: dev-eth0
 * @url: https://www.dev-eth0.de/
 */

#include <Wire.h>
#include <LowPower.h> // https://github.com/rocketscream/Low-Power

#include <RH_RF95.h>
#include <RTClibExtended.h>
#include <Adafruit_seesaw.h>
#include <avr/power.h>

// Unused digital I/O pins 0, 1, 4

#define INT0_PIN 2    //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW

#define RFM95_INT 3
#define RFM95_CS 5
#define RFM95_RST 6
#define RFM95_EN 7

#define SOIL_SENSOR_POWER 8
#define CLOCK_POWER 9

#define CLOCK_STATUS 13

// TODO Remove this status
#define LORA_INIT_STATUS CLOCK_STATUS

#define RF95_FREQ 915.0

#define SENSOR_ID "1"   // Each slave sensor must ahve a unique ID
#define SLEEP 1
#define FORCE_CLOCK_RESET 0
#define SOIL_SENSOR_DELAY 2000
#define AREF_VOLTAGE_X1000 1080L    // Used to get battery voltage in get_bandgap()
#define SOIL_SENSOR_ADDR 0x36

// Tx should not use the Serial interface except for debugging
#define DEBUG 1
#if DEBUG
#define IO(x) do { x; } while (0)
#else
#define IO(x)
#endif

// from  http://heliosoph.mit-links.info/arduino-powered-by-capacitor-reducing-consumption/
//defines for DIDR0 setting
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// create variables to restore the SPI & ADC register values after shutdown
byte keep_ADCSRA = ADCSRA;
byte keep_SPCR = SPCR;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Real time clock
RTC_DS3231 RTC;      // we are using the DS3231 RTC

// Stemma soil sensor
Adafruit_seesaw ss;

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

char date_time_str[32];

char *iso8601_date_time(DateTime t) {
  char val[8];
  date_time_str[0] = '\0';
  strncat(date_time_str, itoa(t.year(), val, 10), sizeof(date_time_str));
  strncat(date_time_str, "-", sizeof(date_time_str));
  if (t.month() < 10) strncat(date_time_str, "0", sizeof(date_time_str));
  strncat(date_time_str, itoa(t.month(), val, 10), sizeof(date_time_str));
  strncat(date_time_str, "-", sizeof(date_time_str));
  if (t.day() < 10) strncat(date_time_str, "0", sizeof(date_time_str));
  strncat(date_time_str, itoa(t.day(), val, 10), sizeof(date_time_str));
  strncat(date_time_str, "T", sizeof(date_time_str));
  if (t.hour() < 10) strncat(date_time_str, "0", sizeof(date_time_str));
  strncat(date_time_str, itoa(t.hour(), val, 10), sizeof(date_time_str));
  strncat(date_time_str, ":", sizeof(date_time_str));
  if (t.minute() < 10) strncat(date_time_str, "0", sizeof(date_time_str));
  strncat(date_time_str, itoa(t.minute(), val, 10), sizeof(date_time_str));
  strncat(date_time_str, ":", sizeof(date_time_str));
  if (t.second() < 10) strncat(date_time_str, "0", sizeof(date_time_str));
  strncat(date_time_str, itoa(t.second(), val, 10), sizeof(date_time_str));

  return date_time_str;
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
  const long InternalReferenceVoltage = 1115L;// Adjust this value to your boards specific internal BG voltage x1000
  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc reference
  // MUX4 MUX3 MUX2 MUX1 MUX0  --> 11110 1.1V (VBG)         -Selects channel 30, bandgap voltage, to measure
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (0 << MUX5) | (1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);

#else
  // For 168/328 boards
  const long InternalReferenceVoltage = AREF_VOLTAGE_X1000; // Adjust this value to your boards specific internal BG voltage x1000
  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
  // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3)
          | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);

#endif

  // TODO Needed?
  // power_adc_enable(); ADCSRA = keep_ADCSRA;  // & throw away the first reading or two…
  
  delay(50);  // Let mux settle a little to get a more stable A/D conversion
  // Start a conversion
  ADCSRA |= _BV(ADSC);
  // Wait for it to complete
  while (((ADCSRA & (1 << ADSC)) != 0))
    ;
  // Scale the value
  uint16_t results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L; // calculates for straight line value

  // Shutdown the ADC
  // shutdown_unused();
  
  return results;
}

// The following saves some extra power by disabling some 
// peripherals I am not using.
void shutdown_unused() {
  // pull up any unused digital pins:
  // pinMode(pin#, INPUT_PULLUP);
  
  // Disable the ADC by setting the ADEN bit (bit 7)  of the
  // ADCSRA register to zero.
  ADCSRA = ADCSRA & B01111111;
  
  // Disable the analog comparator by setting the ACD bit
  // (bit 7) of the ACSR register to one.
  ACSR = B10000000;
  
  // Disable digital input buffers on all analog input pins
  // by setting bits 0-5 of the DIDR0 register to one.
  // Of course, only do this if you are not using the analog 
  // inputs for your project. Later on we'll set up $ and A5 
  // for the I2C bus. A6 and A7 are analog only.
  // Another way to do this: pinMode(A0, INPUT); digitalWrite(A3, HIGH);
  DIDR0 = DIDR0 | B00111111;

  power_timer1_disable();   // (0.12mA) controls PWM 9 & 10, Servo library
  power_timer2_disable();   // (0.12mA) controls PWM 3 & 11
  power_adc_disable();      // disable the clock to the ADC module 

#if 0
  power_twi_disable();    // (0.18mA) I don’t usually turn off I2C because I use the bus frequently
#endif
  // also see: https://harizanov.com/2013/02/power-saving-techniques-on-the-atmega32u4/
#if 0
  // Shutting down ADC & SPI requires you to wake them up again if you need them:
  // Before any SD card data saving:
  power_spi_enable(); SPCR=keep_SPCR;  delay(1); 
#endif
  // Before ADC reading:
  // power_adc_enable(); ADCSRA = keep_ADCSRA;  // & throw away the first reading or two…
}

void DS3231_setup(bool initial_call) 
{
  if (initial_call) {
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
  }
  
  //clear any pending alarms
  RTC.armAlarm(1, false);
  RTC.clearAlarm(1);
  RTC.alarmInterrupt(1, false);
  RTC.armAlarm(2, false);
  RTC.clearAlarm(2);
  RTC.alarmInterrupt(2, false);

  // Set BBSQW (Battery Backup SQW) so that the alarm can generate
  // an interrupt when batery powered. jhrg 3/8/19
  // See https://forum.arduino.cc/index.php?topic=549372.0
  // NB: For this to work, it must be called before DS3231_OFF, since
  // that call sets INTCN. Any other call to writeSqwPinMode clears INTCN,
  // including the BBSQW call, and clearing INTCN disables interrupts.

  // Set SQW oscillator to OFF when in battery mode (!ESOC)
  RTC.writeSqwPinMode(DS3231_OFF);
  Serial.print(F("RTC readSqwPinMode inside configure: "));
  Serial.println(RTC.readSqwPinMode(), HEX);

  Serial.print(F("RTC read(DS3231_CONTROL) inside configure: "));
  Serial.println(RTC.read(DS3231_CONTROL), HEX);
  
  byte control_value = RTC.read(DS3231_CONTROL);
  RTC.write(DS3231_CONTROL, control_value | DS3231_BBSQW);

  Serial.print(F("RTC read(DS3231_CONTROL) inside configure: "));
  Serial.println(RTC.read(DS3231_CONTROL), HEX);
  
  // Check the 32kHz osc.
  if (RTC.getEN32kHz()) {
    IO(Serial.println(F("32kHz osc running")));
    RTC.clearEN32kHz();
    if (RTC.getEN32kHz()) {
      IO(Serial.println(F("32kHz osc still running... FAIL")));
      blink_times(CLOCK_STATUS, 20 /* Hz */, 0); // 20 Hz blink forever
    }
  }

  IO(Serial.println(F("32kHz osc not running")));

  // RTC setup success, status on.
  IO(Serial.println(F("RTC init OK!")));
}


// Configure the radio. If an error is detected, blink the status LED.
// If an error is detected, this function does not return.
void lora_setup() {
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
  // If frequency set fails, blink at 2Hz forever.
  if (!rf95.setFrequency(RF95_FREQ)) {
    IO(Serial.println(F("setFrequency failed")));
    blink_times(LORA_INIT_STATUS, 20, 0); // 2 Hz blink
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
  rf95.setCodingRate4(8);

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  // LORA setup success, status on.
  IO(Serial.println(F("LoRa radio init OK!")));
  digitalWrite(LORA_INIT_STATUS, HIGH);
}

void setup() {
  // Turn off unused features; init code will turn on things that are used
  // shutdown_unused();
  
  // Configure Interrupt Pin
  //pinMode(INT0_PIN, INPUT_PULLUP);
  pinMode(INT0_PIN, INPUT);
  pinMode(CLOCK_STATUS, OUTPUT);
  pinMode(CLOCK_POWER, OUTPUT);
  pinMode(SOIL_SENSOR_POWER, OUTPUT);

  pinMode(0, OUTPUT); digitalWrite(0, LOW);
  pinMode(1, OUTPUT); digitalWrite(1, LOW);
  pinMode(4, OUTPUT); digitalWrite(4, LOW);
  
#if DEBUG 
  // Start the serial port
  while (!Serial) ;
  Serial.begin(9600);
  Serial.println(F("Alarm Test"));
#endif

  digitalWrite(INT0_PIN, HIGH);
  digitalWrite(CLOCK_POWER, HIGH);
  
  // Configures the clock and sets the status HIGH
  DS3231_setup(true);

  // This is the only SPI device, so set the CS LOW here
  digitalWrite(RFM95_CS, LOW);
  
  // Power on the lora
  digitalWrite(RFM95_EN, HIGH);
  lora_setup();
  // Power down the radio
  digitalWrite(RFM95_EN, LOW);
  
  digitalWrite(SOIL_SENSOR_POWER, HIGH);

  // Initialize the soil sensor
  if (!ss.begin(SOIL_SENSOR_ADDR)) {
    Serial.println("ERROR! seesaw not found");
    blink_times(LORA_INIT_STATUS, 30, 0); // 2 Hz blink
  } 
  else {
    Serial.print("seesaw started! version: ");
    Serial.println(ss.getVersion(), HEX);
  }
}

// Send basic information about the sensor plus the information in time_stamp, etc.
// Wait for a response and record the SNR and RSSI of the response (used in the
// next send_packet() transmission).
void send_packet(float clock_temp, float soil_temp, uint16_t soil_moisture) {
  IO(Serial.println(F("Sending to rf95_server")));

  static uint32_t packetnum = 0;  // packet counter, we increment per xmission
  static int16_t rssi = 0;        // rssi and snr are read from the ACK received and sent as part of the next packaet
  static int32_t snr = 0;

  char packet[64];
  char *time_stamp = iso8601_date_time(RTC.now());
  
  // Send a message to rf95_server
  // The RSSI and SNR are for the most recent ACK received from the server in response to
  // the client's message. The message is '<packetnum>,<Vcc>,<rssi>,<snr>,<temp>'.
  // Vcc is X100 (see get_bandgap()) and temp is X100 as well.
  char val[8];
  packet[0] = '\0';

  strncpy(packet, time_stamp, sizeof(packet));  // 19
  strncat(packet, ",", sizeof(packet));

  strncat(packet, SENSOR_ID, sizeof(packet)); // 1
  strncat(packet, ",", sizeof(packet));

  strncat(packet, itoa(++packetnum, val, 10), sizeof(packet)); // 5+
  strncat(packet, ",", sizeof(packet));

  strncat(packet, itoa(get_bandgap(), val, 10), sizeof(packet)); // 3
  strncat(packet, ",", sizeof(packet));

  strncat(packet, itoa(rssi, val, 10), sizeof(packet)); // 6
  strncat(packet, ",", sizeof(packet));

  strncat(packet, itoa(snr, val, 10), sizeof(packet)); // 3
  strncat(packet, ",", sizeof(packet));

  strncat(packet, itoa((int) (clock_temp * 100), val, 10), sizeof(packet));
  strncat(packet, ",", sizeof(packet));
  
  strncat(packet, itoa((int) (soil_temp * 100), val, 10), sizeof(packet));
  strncat(packet, ",", sizeof(packet));

  strncat(packet, itoa(soil_moisture, val, 10), sizeof(packet));
  
  rf95.send((uint8_t *) packet, strnlen(packet, sizeof(packet)) + 1);

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

void wake_up() {
  // wake up again
  IO(Serial.println(F("Woke up this morning...")));
  // Remove the interrupt here. If this is done in the main loop, this function will be called
  // many times
  detachInterrupt(0);
}

void loop() {
  // Set alarm
  IO(Serial.println(F("Setting alarm")));
  
  // Set alarm every minute at the 15 second mark
  RTC.setAlarm(ALM1_MATCH_SECONDS, 15, 0, 0, 0);

  RTC.armAlarm(1, true);
  RTC.alarmInterrupt(1, true);

  digitalWrite(CLOCK_STATUS, HIGH);
  Serial.println(F("After reconfiguration"));
  Serial.print(F("Alarm 1: "));
  Serial.println(RTC.isArmed(1));
  Serial.print(F("Alarm 2: "));
  Serial.println(RTC.isArmed(2));
  Serial.print(F("RTC Status: "));
  Serial.println(RTC.readSqwPinMode(), HEX);
  Serial.print(F("RTC 32kHz: "));
  Serial.println(RTC.getEN32kHz());
  Serial.flush();
  
  // wake_up() sets AlarmFlag
  // use interrupt 0 (pin 2) and run function wake_up when pin 2 gets LOW
  attachInterrupt(digitalPinToInterrupt(INT0_PIN), wake_up, LOW); 
  digitalWrite(CLOCK_STATUS, LOW); //switch-off the led for indicating that we enter the sleep mode
  
  pinMode(CLOCK_POWER, INPUT);
  digitalWrite(CLOCK_POWER, LOW);
  
  pinMode(SOIL_SENSOR_POWER, INPUT);
  digitalWrite(SOIL_SENSOR_POWER, LOW);     

#if SLEEP
  //SLEEP_MODE_PWR_DOWN
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); //arduino enters sleep mode here
#endif

#if DEBUG
  digitalWrite(LORA_INIT_STATUS, HIGH);
#endif

  pinMode(CLOCK_POWER, OUTPUT);
  digitalWrite(CLOCK_POWER, HIGH);
  
  pinMode(SOIL_SENSOR_POWER, OUTPUT);
  digitalWrite(SOIL_SENSOR_POWER, HIGH);
  
  Serial.println(F("After wakeup"));
  Serial.print(F("Alarm 1: "));
  Serial.println(RTC.isArmed(1));
  Serial.print(F("Alarm 2: "));
  Serial.println(RTC.isArmed(2));
  Serial.print(F("RTC Status: "));
  Serial.println(RTC.readSqwPinMode(), HEX);
  Serial.print(F("RTC 32kHz: "));
  Serial.println(RTC.getEN32kHz());
  Serial.flush();

  Serial.println(iso8601_date_time(RTC.now()));

  // Read from the soil sensor
  uint16_t capread = ss.touchRead(0);
  Serial.print("Capacitive after wake up: ");
  Serial.print(capread);
  Serial.print(", delay: ");
  Serial.println(SOIL_SENSOR_DELAY);

  // A 2s wait seems needed, although the begin() method only uses 0.5s
  delay(SOIL_SENSOR_DELAY);

  float tempC = ss.getTemp();
  capread = ss.touchRead(0);

  Serial.print("Temperature: ");
  Serial.print(tempC);
  Serial.println("C");
  Serial.print("Capacitive: ");
  Serial.println(capread);

  // Power up the lora
  digitalWrite(RFM95_EN, HIGH);

  send_packet(RTC.getTemp(), tempC, capread);

  // Power down the lora
  digitalWrite(RFM95_EN, LOW);
  
#if DEBUG
  digitalWrite(LORA_INIT_STATUS, LOW);
#endif
}
