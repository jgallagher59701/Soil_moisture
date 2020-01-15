
/**
 * Soil sensor prototype number 4
 * James Gallagher <jgallagher@opendap.org>
 */

#include <LowPower.h> // https://github.com/rocketscream/Low-Power
#include <RH_RF95.h>

#define INT0_PIN 2

#define RFM95_INT 3
#define RFM95_CS 5
#define RFM95_RST 6
//#define RFM95_EN 7

#if 0
#define MOSIPin 11
#define MISOPin 12
#define SCKPin 13
#endif

#define LEVEL 4
#define SENSOR_ID "2"
#define RF95_FREQ 915.0
#define TX_LEVEL 23
#define AREF_VOLTAGE_X1000 1080L    // Used to get battery voltage in get_bandgap()

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void wake_up() {
  // Remove the interrupt here. If this is done in the main loop, this function will be called
  // many times
  detachInterrupt(0);
}

// Blink the builtin LED \c t times with a 1s delay between each blink.
// There is a 0.5s wait between blinks.
void blink_times(int t) {
  for (int i = 0; i < t; ++i) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    LowPower.idle(SLEEP_500MS, ADC_ON, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF,
                  SPI_ON, USART0_OFF, TWI_ON);
  }
}

// Like blink, but never returns. Blinks, then waits for 1s, then blinks again.
// Uses a faster blink rate (120ms instead of 500ms for \c blink_times().
void blink_error(int t) {
  while (true) {
    for (int i = 0; i < t; ++i) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      LowPower.idle(SLEEP_120MS, ADC_ON, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF,
                    SPI_ON, USART0_OFF, TWI_ON);
    }
    LowPower.idle(SLEEP_1S, ADC_ON, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF,
                  SPI_ON, USART0_OFF, TWI_ON);

  }
}

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

  delay(50);  // Let mux settle a little to get a more stable A/D conversion
  // Start a conversion
  ADCSRA |= _BV(ADSC);
  // Wait for it to complete
  while (((ADCSRA & (1 << ADSC)) != 0))
    ;
  // Scale the value
  uint16_t results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L; // calculates for straight line value
  
  return results;
}

// Configure the radio. If an error is detected, blink the status LED.
// If an error is detected, this function does not return.
void lora_setup() {
  // LED to show the LORA radio has been configured - turn on once the LORA is setup
  blink_times(2);

  digitalWrite(RFM95_CS, LOW);

  // LORA manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Defaults for RFM95 after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5,
  // Sf = 128chips/symbol, CRC on. If the init fails, LORA status LED blinks at
  // 10Hz
  while (!rf95.init()) {
    Serial.println(F("1 Init fail"));
    Serial.flush();
    blink_error(2);
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM.
  // If frequency set fails, blink at 2Hz forever.
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println(F("2 freq fail"));
    Serial.flush();
    blink_error(3);
  }

  // None of these return error codes.
  // Setup Spreading Factor (chips/symbol) (n = 6 ~ 12, where Sf=2^n (eg 6 --> 2^6 == 64 chips/sym)
  rf95.setSpreadingFactor(7);

  // Setup BandWidth, option: 7800,10400,15600,20800,31200,41700,62500,125000,250000,500000
  // Higher == higher data rates
  rf95.setSignalBandwidth(125000);

  // Setup Coding Rate:5(4/5),6(4/6),7(4/7),8(4/8) (higher == better error correction)
  rf95.setCodingRate4(6);

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(TX_LEVEL, false);

  // LORA setup success, status on.
  Serial.println(F("RF95 success"));
  Serial.flush();
  blink_times(3);
}

void setup() {
  pinMode(INT0_PIN, INPUT_PULLUP);
  //pinMode(INT0_PIN, INPUT);

  pinMode(RFM95_INT, INPUT_PULLUP);
  pinMode(RFM95_CS, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  while (!Serial) ;
  Serial.begin(9600);
  Serial.println(F("Power Test"));

  lora_setup();
}

/////////////////////////////// End setup ///////////////////////////////

void power_up_lora()
{  
  digitalWrite(RFM95_CS, LOW);

  rf95.available();
}

void power_down_lora()
{
  rf95.sleep();

  digitalWrite(RFM95_CS, HIGH);
}

uint32_t packetnum = 0;  // packet counter, we increment per xmission
int16_t rssi = 0;        // rssi and snr are read from the ACK received and sent as part of the next packaet
int32_t snr = 0;

// Send basic information about the sensor plus the information in time_stamp, etc.
// Wait for a response and record the SNR and RSSI of the response (used in the
// next send_packet() transmission).
//
// time_stamp Static string returned by iso8601_date_time(RTC.now());
// clock_temp Temperature read from the RTC
// soil_temp Temperature read from the soil sensor
// soil_moisture Soil moisture from the soil sensor
void send_packet(char *time_stamp, float clock_temp, float soil_temp, uint16_t soil_moisture) {
  char packet[64];
  
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

  blink_times(1);
  
  rf95.send((uint8_t *) packet, strnlen(packet, sizeof(packet)) + 1);

  Serial.println(F("before rf95.waitPacketSent()"));
  Serial.flush();

  rf95.waitPacketSent();

  Serial.println(F("after rf95.waitPacketSent()"));
  Serial.flush();

  uint8_t buf[64 /*RH_RF95_MAX_MESSAGE_LEN*/];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(1000)) {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len)) {
      // Update rssi and snr
      rssi = rf95.lastRssi();
      snr = rf95.lastSNR();

      Serial.println(F("RF95 reply... "));
      Serial.println((char*)buf);
      Serial.flush();

      blink_times(2);
     } 
    else {
      rssi = 0;
      snr = 0;

      Serial.println(F("RF95 no reply"));
      Serial.flush();

      blink_times(3);
    }
  } 
  else {
    Serial.println(F("RF95 reply timeout"));
    Serial.flush();
   
   blink_times(4);
  }
}

void loop() {
#if LEVEL == 0
  // do nothing

#elif LEVEL == 1
  // No sleeping here. measure just the atmega328p power use
  digitalWrite(LED_BUILTIN, HIGH);

  delay(5);

  digitalWrite(LED_BUILTIN, LOW);

  delay(5000);

#elif LEVEL == 2
  Serial.println(F("Watch Dog timer sleep Test"));
  Serial.flush();
  
  // Watch Dog timer sleep for 8s
  digitalWrite(LED_BUILTIN, HIGH);

  delay(5);

  digitalWrite(LED_BUILTIN, LOW);

  while (digitalRead(INT0_PIN) == LOW) ;

#if 0
  LowPower.idle(SLEEP_8S, ADC_ON, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF,
                SPI_ON, USART0_OFF, TWI_ON);
#elif 0
  LowPower.adcNoiseReduction(SLEEP_8S, ADC_ON, TIMER2_OFF);
#elif 0
  LowPower.powerSave(SLEEP_8S, ADC_ON, BOD_ON, TIMER2_OFF);
#elif 1
  LowPower.powerStandby(SLEEP_8S, ADC_ON, BOD_ON);
#endif

  delay(500);

#elif LEVEL == 3
  Serial.println(F("External interrupt sleep Test"));
  Serial.flush();

  // deep sleep - wakeup using an external interrupt. Move a jumper from
  // Vcc to GND to trigger the interrupt. Uses an internal (50k) PU resistor.
  digitalWrite(LED_BUILTIN, HIGH);

  delay(5);

  digitalWrite(LED_BUILTIN, LOW);

  while (digitalRead(INT0_PIN) == LOW) ;

  // 5s to move the jumper back to Vcc...
  delay(5000);

  // use interrupt 0 (pin 2) and run function wake_up when pin 2 gets LOW
  attachInterrupt(digitalPinToInterrupt(INT0_PIN), wake_up, LOW);

  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); //arduino enters sleep mode here

  Serial.println(F("Woke up!"));
  Serial.flush();

  delay(500);

#elif LEVEL == 4
  Serial.println(F("LoRa sleep Test"));
  Serial.flush();

  // deep sleep with lora - wakeup using an external interrupt. Move a jumper from
  // Vcc to GND to trigger the interrupt. Uses an internal (50k) PU resistor.
  blink_times(1);

  power_up_lora();

  //send_packet(char *time_stamp, float clock_temp, float soil_temp, uint16_t soil_moisture);
  send_packet("1970-01-01T00:00:00", 0.0, 0.0, 17);
  
  power_down_lora();
  
  while (digitalRead(INT0_PIN) != HIGH) ;

  // 5s to move the jumper back to Vcc...
  delay(5000);

  // use interrupt 0 (pin 2) and run function wake_up when pin 2 gets LOW
  attachInterrupt(digitalPinToInterrupt(INT0_PIN), wake_up, LOW);

  LowPower.powerDown(/*SLEEP_4S*/ SLEEP_FOREVER, ADC_OFF, BOD_OFF); //arduino enters sleep mode here

  Serial.println(F("Woke up!"));
  Serial.flush();

  delay(500);
#endif
}
