//
// Simple lora client. Uses Simple lora server.
//
// Modified from the pro-mini version to work with the Rocket Scream 
// Mini Ultra Pro, V3, which includes a LoRa module on the RS board.
//
// Based on LoRa Simple Yun Client by Edwin Chen <support@dragino.com>,
// Dragino Technology Co., Limited
//
// James Gallagher <jgallagher@opendap.org>
// 7/26/20

#include <SPI.h>
#include <RH_RF95.h>

//#include <LowPower.h>

// Pin assignments

#define RFM95_INT 2     // RF95 Interrupt
#define RFM95_CS 5      // RF95 SPI CS

#define SD_PWR 3        // HIGH == power on SD card

#define USE_CAD 9       // If pin 9 is high, use CAD
#define USE_LOOP_DELAY 8  // If pin 8 is high, use a loop delay
#define STATUS_LED 13   // Only for DEBUG mode

#define V_BAT A0

// Constants

#define FREQ 915.0
#define BANDWIDTH 125000
#define SPREADING_FACTOR 7  // sf = 6 - 12 --> 2^(sf)
#define CODING_RATE 5

#define DEBUG 1             // Requires USB
#define Serial SerialUSB    // Needed for RS. jhrg 7/26/20

#define NODE 1
#define EXPECT_REPLY 0

#define WAIT_AVAILABLE 3000 // ms to wait for a response from server
#define TX_INTERVAL 6000 // ms to wait before next transmission
#define CAD_TIMEOUT 3000 // timeout for CAD wait

#define ADC_BITS 12
#define ADC_MAX_VALUE 4096
// Without a loop delay, packets will be sent as fast as possible.
// This should cause maximum collisions.

#if DEBUG
#define IO(x) do { x; } while (false)
#else
#define IO(x)
#endif

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

unsigned int tx_power = 13;   // dBm 5 tp 23 for RF95

void setup()
{

  pinMode(USE_CAD, INPUT_PULLUP);
  pinMode(USE_LOOP_DELAY, INPUT_PULLUP);
  
  pinMode(STATUS_LED, OUTPUT);

  analogReadResolution(ADC_BITS);

  pinMode(SD_PWR, OUTPUT);
  digitalWrite(SD_PWR, LOW);
  
  IO(Serial.begin(9600));
  IO(while (!Serial)); // Wait for serial port to be available

#if 0
  // I'm not sure if manual reset is supported by the RS. jhrg 7/23/20
  // LORA manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(15);
  // LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  digitalWrite(RFM95_RST, HIGH);
  delay(15);
  //LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
#endif

  IO(Serial.println(F("Start LoRa Client")));

  if (!rf95.init()) {
    IO(Serial.println(F("LoRa init failed.")));
    // Change this to inifinite blink
    while (true) ;
  }

  // Setup ISM frequency
  rf95.setFrequency(FREQ);

  // Setup BandWidth, option: 7800,10400,15600,20800,31200,41700,62500,125000,250000,500000
  // Lower BandWidth for longer distance.
  rf95.setSignalBandwidth(BANDWIDTH);

  // Setup Power,dBm
  rf95.setTxPower(tx_power);

  // Setup Spreading Factor (6 ~ 12)
  rf95.setSpreadingFactor(SPREADING_FACTOR);

  // Setup Coding Rate:5(4/5),6(4/6),7(4/7),8(4/8)
  rf95.setCodingRate4(CODING_RATE);

  // rf95.setCADTimeout(CAD_TIMEOUT);
}

void yield(unsigned long ms_delay)
{
  unsigned long start = millis();
  while ((millis() - start) < ms_delay)
    yield();
}

// Function created to obtain chip's actual Vcc voltage value, using internal bandgap reference
// This demonstrates ability to read processors Vcc voltage and the ability to maintain A/D
// calibration with changing Vcc. Now works for 168/328 and mega boards.
// Thanks to "Coding Badly" for direct register control for A/D mux
// 1/9/10 "retrolefty"

/**
 * @note this version assumes that a voltage divider reduces Vbat by 1/4.3
 * @return The battery voltage x 100 as an int
 */
int get_bat_v()
{
  // voltage divider v_bat = 4.3 * vadc
  // vadc = (raw / 4096)
  int raw = analogRead(V_BAT);
  return 430 * (raw / (float)ADC_MAX_VALUE);  // voltage * 100
}

void loop()
{
  IO(Serial.print(F("Sending to LoRa Server.. ")));
  static unsigned long last_tx_time = 0;
  static unsigned long message = 0;

  // Send a message to LoRa Server

  ++message;

  // Test: SD_PWR on for even messages
  if (message % 2 == 0)
    digitalWrite(SD_PWR, HIGH);
  else
    digitalWrite(SD_PWR, LOW);
    
  digitalWrite(STATUS_LED, HIGH);

  if (digitalRead(USE_CAD) == HIGH)
    rf95.setCADTimeout(CAD_TIMEOUT);
  else
    rf95.setCADTimeout(0);

  uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
  snprintf((char*)data, sizeof(data), "Hello, node %d, message %ld, tx time %ld ms, battery %d", NODE, message, last_tx_time, get_bat_v());

  IO(Serial.println((const char*)data));

  unsigned long start_time = millis();

  rf95.send(data, sizeof(data));  // This may block for up to CAD_TIMEOUT
  rf95.waitPacketSent();  // Block until packet sent

  unsigned long end_time = millis();
  last_tx_time = end_time - start_time;   // last_tx_time used next iteration

#if EXPECT_REPLY
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(WAIT_AVAILABLE))
  {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len))
    {
      IO(Serial.print(F("got reply: ")));
      IO(Serial.println((char*)buf));
      IO(Serial.print(F("RSSI: ")));
      IO(Serial.println(rf95.lastRssi(), DEC));
    }
    else
    {
      IO(Serial.println(F("receive failed")));
    }
  }
  else
  {
    IO(Serial.println(F("No reply, is LoRa server running?")));
  }
#endif  // EXPECT_REPLY

  digitalWrite(STATUS_LED, LOW);

  if (digitalRead(USE_LOOP_DELAY) == HIGH) {
    unsigned long elapsed_time = millis() - start_time;
    yield(max(TX_INTERVAL - elapsed_time, 0)); // wait here for upto TX_INTERVAL ms
  }
}
