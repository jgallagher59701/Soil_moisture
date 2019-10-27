/**
 * Soil Moisture sensor and LoRa transceiver.
 * 
 * To test this with the echoing LoRa receiver code, use
 * 'ssh root@10.130.1.1 "telnet localhost 6571"'
 * This will ssh to the dragino as root (pw 'dragino') and then
 * run telnet to localhost on port 6571. teh Dragino uses that
 * port as a gateway between code running in the Yun (linux) and
 * arduino (which is running the 'echoing' sketch and is connected 
 * to the LoRa,
 * 
 * James Gallagher <jgallagher@opendap.org>
 * 10/21/19
 */

#include <Wire.h>
#include "LowPower.h"

#include <RH_RF95.h>
// DS3231.h doesn't have TimeSpan, but might use less space
#include <RTClibExtended.h>
// Might edit this and make a lib that has only what the soil sensor needs.
// Look in Documents/Arduino/libraries 
#include <Adafruit_seesaw.h>

#define RFM95_INT 3
#define RFM95_CS 5
#define RFM95_RST 6

#define CLOCK_BAT_VOLTAGE A0
#define TMP36 A1

// Was LED_BUILTIN
// The status LED will only be used for errors in production code
#define STATUS 9 

#define RF95_FREQ 915.0

#define SENSOR_ID "1"   // Each slave sensor must ahve a unique ID
#define AREF_VOLTAGE_X1000 1080L    // Used to get battery voltage in get_bandgap()
#define FORCE_CLOCK_RESET 0

#define DS3231_CLOCK_ADR 0x38
#define SOIL_SENSOR_ADDR 0x36

// Tx should not use the Serial interface except for debugging
#define DEBUG 1

#if DEBUG
#define IO(x) do { x; } while (0)
#else
#define IO(x)
#endif

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Singleton real time clock
RTC_DS3231 RTC;

Adafruit_seesaw soil_moisture;

///
/// General functions
///

// Blink 2, 3, ..., times to indicate different componenets starting.
// Blink twice at the start of initialization, once at the end, or
// forever if an error occurs. The SAMPLE_STATUS and STARTED/COMPLETED
// values are used only for testing
#define LORA_STATUS 2
#define CLOCK_STATUS 3
#define SOIL_MOISTURE_STATUS 4
#define SAMPLE_STATUS 5

#define START 1
#define COMPLETED 1
#define ERROR_OCCURRED 0

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
      // LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
      digitalWrite(pin, LOW);
      delay(quarter_second);
      // LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
    }
    delay(one_second);
    //LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
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

  //delay(50);  // Let mux settle a little to get a more stable A/D conversion
  LowPower.powerDown(SLEEP_60MS, ADC_ON, BOD_OFF);
  // Start a conversion
  ADCSRA |= _BV(ADSC);
  // Wait for it to complete
  while (((ADCSRA & (1 << ADSC)) != 0)) ;
  // Scale the value
  uint16_t results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L; // calculates for straight line value
  
  return results;
}

// Given a DateTime instance, return a pointer to static string that holds
// an ISO 8601 print representation of the object.

char *iso8601_date_time(DateTime t) {
  static char date_time_str[32];
  
  char val[12];
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

///
/// Set up functions. Configure the MCU or a peripheral
///

// Configure I/O ports so that by default they are in a low-power state.
// other initialization code will set up the ports that are used as
// needed.
void port_setup() {
    // Turn off unused features; init code will turn on things that are used
  // The following saves some extra power by disabling some 
  // peripherals I am not using.
  
  // Disable the ADC by setting the ADEN bit (bit 7)  of the
  // ADCSRA register to zero.
  // ADCSRA = ADCSRA & B01111111;
  
  // Disable the analog comparator by setting the ACD bit
  // (bit 7) of the ACSR register to one.
  // ACSR = ACSR | B10000000;
  
  // Disable digital input buffers on all analog input pins
  // by setting bits 0-5 of the DIDR0 register to one.
  // Of course, only do this if you are not using the analog 
  // inputs for your project.
  // NB: DIDR == Digital Input Disable Register
  DIDR0 = DIDR0 | B00111111; 

  // Set all of the digital pins to input and enable the internal 
  // pullup resistors. This consumes less power when the AVR is running
  // but may consume more if it is sleeping.
  // https://arduino.stackexchange.com/questions/16424/deviation-of-power-consumption-between-datasheet-and-real-world

  // Also see: https://www.arduino.cc/en/Reference/PortManipulation
  // DDRD pins 0 - 7
  DDRD = DDRD & B00000011;  // 2 to 7 as inputs; leaves pins 0 & 1, which are RX & TX
  //DDRB pins 8 - 13 
  DDRB = DDRB & B11000000;  // 8 - 13 as inputs; two high bits are not used

  // PORTD Sets the read/write state. Inputs that are set to Write have the pull-up
  // resistor enabled
  PORTD = PORTD | B11111100;
  PORTB = PORTB | B00111111;
}

// Configure the radio. If an error is detected, blink the status LED.
// If an error is detected, this function does not return.
void lora_setup() {
  // LED to show the LORA radio has been configured - turn on once the LORA is setup
  blink_times(STATUS, LORA_STATUS, START);
  
  digitalWrite(RFM95_CS, LOW);
  
  // LORA manual reset
  digitalWrite(RFM95_RST, LOW);
  //delay(10);
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  digitalWrite(RFM95_RST, HIGH);
  //delay(10);
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);

  // Defaults for RFM95 after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5,
  // Sf = 128chips/symbol, CRC on. If the init fails, LORA status LED blinks at
  // 10Hz
  while (!rf95.init()) {
    IO(Serial.println(F("LoRa radio init failed")));
    blink_times(STATUS, LORA_STATUS, ERROR_OCCURRED); //
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM.
  // If frequency set fails, blink at 2Hz forever.
  if (!rf95.setFrequency(RF95_FREQ)) {
    IO(Serial.println(F("setFrequency failed")));
    blink_times(STATUS, LORA_STATUS, ERROR_OCCURRED);
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
  blink_times(STATUS, LORA_STATUS, COMPLETED);
}

// Can be called both when the clock is first powered and when it's been running
// by setting 'initial_call' to true or false.
void clock_setup(bool initial_call) 
{
  if (initial_call) {
    blink_times(STATUS, CLOCK_STATUS, START);
    // The DS3231 requires the Wire library
    Wire.begin();
    
    if (!RTC.begin()) {
      IO(Serial.println(F("Couldn't find RTC")));
      blink_times(STATUS, CLOCK_STATUS, ERROR_OCCURRED);
    }

    if (RTC.lostPower() || FORCE_CLOCK_RESET) {
      IO(Serial.println(F("RTC lost power, lets set the time!")));
      // following line sets the RTC to the date & time this sketch was compiled
      RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
      // This line sets the RTC with an explicit date & time, for example to set
      // January 21, 2014 at 3am you would call:
      // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
      blink_times(STATUS, CLOCK_STATUS, ERROR_OCCURRED);
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
  IO(Serial.print(F("RTC readSqwPinMode inside configure: ")));
  IO(Serial.println(RTC.readSqwPinMode(), HEX));

  IO(Serial.print(F("RTC read(DS3231_CONTROL) inside configure: ")));
  IO(Serial.println(RTC.read(DS3231_CONTROL), HEX));
  
  byte control_value = RTC.read(DS3231_CONTROL);
  RTC.write(DS3231_CONTROL, control_value | DS3231_BBSQW);

  IO(Serial.print(F("RTC read(DS3231_CONTROL) inside configure: ")));
  IO(Serial.println(RTC.read(DS3231_CONTROL), HEX));
  
  // Check the 32kHz osc.
  if (RTC.getEN32kHz()) {
    IO(Serial.println(F("32kHz osc running")));
    RTC.clearEN32kHz();
    if (RTC.getEN32kHz()) {
      IO(Serial.println(F("32kHz osc still running... FAIL")));
      blink_times(STATUS, 20 /* Hz */, 0); // 20 Hz blink forever
    }
  }

  IO(Serial.println(F("32kHz osc not running")));

  // RTC setup success, status on.
  IO(Serial.println(F("RTC init OK!")));
  
  // Only show status on the initial call
  if (initial_call) {
    blink_times(STATUS, CLOCK_STATUS, COMPLETED);
  }
}

void soil_moisture_setup() {
  // Initialize the soil sensor
  blink_times(STATUS, SOIL_MOISTURE_STATUS, START);
  
  if (!soil_moisture.begin(SOIL_SENSOR_ADDR)) {
    IO(Serial.println(F("ERROR! seesaw not found")));
    blink_times(STATUS, SOIL_MOISTURE_STATUS, ERROR_OCCURRED);
  } 
  else {
    IO(Serial.print(F("seesaw started! version: ")));
    IO(Serial.println(soil_moisture.getVersion(), HEX));
  }

    blink_times(STATUS, SOIL_MOISTURE_STATUS, COMPLETED);
}

void setup() {
  port_setup();

  pinMode(STATUS, OUTPUT);  // The LED status indicator
  pinMode(A0, INPUT); // analog; DS3231 battery voltage
  pinMode(A1, INPUT); // analog; TMP36 temp sensor

#if DEBUG 
  // Start the serial port
  while (!Serial) ;
  Serial.begin(9600);
  Serial.println(F("boot"));
#endif

  clock_setup(true);

  soil_moisture_setup();
  
  lora_setup();
}

// Send basic information about the sensor plus the information in time_stamp, etc.
// Wait for a response and record the SNR and RSSI of the response (used in the
// next send_packet() transmission).
//
// TODO use snprintf()
//
// Packet sent: Time, sensor ID, packet num, sensor battery volts, clock temp, clock battery volts, 
//              rssi of last ack, snr of last ack, soil temp, soil moisture

void send_packet(char *time_stamp, uint16_t  clock_temp, uint16_t clock_bat_volts, uint16_t sensor_bat_volts, int16_t soil_temp, uint16_t soil_moisture) {
  static uint32_t packetnum = 0;  // packet counter, we increment per xmission
  static int16_t rssi = 0;        // rssi and snr are read from the ACK received and sent as part of the next packaet
  static int32_t snr = 0;

  IO(Serial.println(F("Sending to rf95_server")));

  char packet[64];
  
  // Send a message to rf95_server
  // The RSSI and SNR are for the most recent ACK received from the server in response to
  // the client's message. The message is '<packetnum>,<Vcc>,<rssi>,<snr>,<temp>'.
  // Vcc is X100 (see get_bandgap()) and temp is X100 as well.
  char val[8];
  packet[0] = '\0';

  strncpy(packet, time_stamp, sizeof(packet)-1);  // 19
  strncat(packet, ",", sizeof(packet)-1);

  strncat(packet, SENSOR_ID, sizeof(packet)-1); // 1 - 4
  strncat(packet, ",", sizeof(packet)-1);

  strncat(packet, itoa(++packetnum, val, 10), sizeof(packet)-1); // 5+
  strncat(packet, ",", sizeof(packet)-1);

  strncat(packet, itoa(sensor_bat_volts, val, 10), sizeof(packet)-1); // 3
  strncat(packet, ",", sizeof(packet)-1);

  strncat(packet, itoa(clock_temp, val, 10), sizeof(packet)-1);
  strncat(packet, ",", sizeof(packet)-1);
  
  strncat(packet, itoa(clock_bat_volts, val, 10), sizeof(packet)-1);
  strncat(packet, ",", sizeof(packet)-1);
  
  strncat(packet, itoa(rssi, val, 10), sizeof(packet)-1); // 6
  strncat(packet, ",", sizeof(packet));

  strncat(packet, itoa(snr, val, 10), sizeof(packet)-1); // 3
  strncat(packet, ",", sizeof(packet)-1);

  strncat(packet, itoa(soil_temp, val, 10), sizeof(packet)-1);
  strncat(packet, ",", sizeof(packet)-1);

  strncat(packet, itoa(soil_moisture, val, 10), sizeof(packet)-1);

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
      IO(Serial.print(F("SNR: "))); 
      IO(Serial.println(snr, DEC));
      IO(Serial.flush());
    } 
    else {
      rssi = 0;
      snr = 0;

      IO(Serial.println(F("Receive failed")));
      IO(Serial.flush());
    }
  } 
  else {
    IO(Serial.println(F("No reply, is there a listener around?")));
    IO(Serial.flush());
  }
}

void set_alarm() {
  // setAlarm(Ds3231_ALARM_TYPES_t alarmType, byte seconds, byte minutes, byte hours, byte daydate);
  // or
  // setAlarm(Ds3231_ALARM_TYPES_t alarmType, byte minutes, byte hours, byte daydate);
  
  // Set alarm every minute at the 15 second mark
  RTC.setAlarm(ALM1_MATCH_SECONDS, 15, 0, 0, 0);

  RTC.armAlarm(1, true);
  RTC.alarmInterrupt(1, true);

  IO(Serial.println(F("After reconfiguration")));
  IO(Serial.print(F("Alarm 1: ")));
  IO(Serial.println(RTC.isArmed(1)));
  IO(Serial.print(F("Alarm 2: ")));
  IO(Serial.println(RTC.isArmed(2)));
  IO(Serial.print(F("RTC Status: ")));
  IO(Serial.println(RTC.readSqwPinMode(), HEX));
  IO(Serial.print(F("RTC 32kHz: ")));
  IO(Serial.println(RTC.getEN32kHz()));
  IO(Serial.flush());
}

// Return the temperature of the soil moisture sensor. Not very precise (+/- 1C)
float get_soil_moisture_temp() {
  return soil_moisture.getTemp();
}

// The value of the seesaw capacitive touch pin. An integer between 0 and 1023
// This waits for the sensor to stabilize.
uint16_t get_soil_moisture_value() {
  uint16_t cap_value = 0xFFFF;
  uint16_t new_cap_value = soil_moisture.touchRead(0);
  
  while(cap_value != new_cap_value) {
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
    cap_value = new_cap_value;
    uint16_t new_cap_value = soil_moisture.touchRead(0);
  }

  return cap_value;
}

// Return the TMP36 temperature, as an int16 scaled by 100
// i.e., 19.0 C --> 190
//
// For the TMP36; t = mx + b --> t = (1000 v - 500)/10

// #define ZERO_CROSSING 470
#define ZERO_CROSSING 500

/// Get the temperature from the TM36 sensor.
/// @param v_bat is the power supply voltage * 100 as an integer.
/// @return The temperature * 10 as an integer
int16_t get_tmp36_temp(uint16_t v_bat) {
  IO(Serial.print(F("TMP36 raw: ")));
  IO(Serial.println(analogRead(TMP36)));
  IO(Serial.flush());

#if 0
  // The formula given on the Adafruit page:
  // https://learn.adafruit.com/tmp36-temperature-sensor/using-a-temp-sensor
  // I think 1023.0 is correct; 1024.0 will introduce a slight error.
  (analogRead(TMP36) * ((v_bat * 10)/1024.0)) - ZERO_CROSSING
#endif

  return (int16_t)((analogRead(TMP36) / 1023.0) * (v_bat * 10)) - ZERO_CROSSING;
}

uint16_t get_clock_bat_volatage(uint16_t  v_bat) {
  return (uint16_t)(analogRead(CLOCK_BAT_VOLTAGE) / 1023.0 * v_bat);
}

void loop() {
  blink_times(STATUS, SAMPLE_STATUS, START);

  uint16_t v_bat = get_bandgap();

  send_packet(iso8601_date_time(RTC.now()), (uint16_t)(RTC.getTemp()*100), get_clock_bat_volatage(v_bat), v_bat, get_tmp36_temp(v_bat), get_soil_moisture_value());

  blink_times(STATUS, SAMPLE_STATUS, COMPLETED);

  LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
}
