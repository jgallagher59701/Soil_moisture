/*
  LoRa Simple Client for Arduino :
  Support Devices: LoRa Shield + Arduino 
  
  Example sketch showing how to create a simple messageing client, 
  with the RH_RF95 class. RH_RF95 class does not provide for addressing or
  reliability, so you should only use RH_RF95 if you do not need the higher
  level messaging abilities.

  It is designed to work with the other example LoRa Simple Server
  User need to use the modified RadioHead library from:
  https://github.com/dragino/RadioHead

  modified 16 11 2016
  by Edwin Chen <support@dragino.com>
  Dragino Technology Co., Limited
*/

#include <SPI.h>
#include <RH_RF95.h>
//#include <LowPower.h>

#define RFM95_INT 3
#define RFM95_CS 5
#define RFM95_RST 6

#define FREQ 915.0
#define BANDWIDTH 125000
#define SPREADING_FACTOR 7  // sf = 6 - 12 --> 2^(sf)
#define CODING_RATE 5

#define NODE 2
#define DEBUG 0
#define EXPECT_REPLY 1
#define WAIT_AVAILABLE 3000 // ms to wait for a response from server
#define TX_INTERVAL 6000 // ms to wait before next transmission
#define CAD_TIMEOUT 3000 // timeout for CAD wait

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
  IO(Serial.begin(9600));
#if 0
  while (!Serial) ; // Wait for serial port to be available
#endif

  // LORA manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(15);
  // LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  digitalWrite(RFM95_RST, HIGH);
  delay(15);
  //LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);

  Serial.println("Start LoRa Client");
  
  if (!rf95.init()) {
    IO(Serial.println(F("LoRa init failed.")));
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

  rf95.setCADTimeout(CAD_TIMEOUT);
}

void loop()
{
  IO(Serial.println(F("Sending to LoRa Server")));
  static unsigned long last_tx_time = 0;
  static unsigned long message = 0;
  
  // Send a message to LoRa Server
  
  ++message;
  unsigned long start_time = millis();
  
  uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
  snprintf(data, sizeof(data), "Hello, this is device %d, message %ld, tx time %ld ms", NODE, message, last_tx_time);
  rf95.send(data, sizeof(data));  // This may block for up to CAD_TIMEOUT
  
  rf95.waitPacketSent();  // Block until packet sent
  
  unsigned long end_time = millis();
  last_tx_time = end_time - start_time;   // last_tx_time used next iteration

  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

#if EXPECT_REPLY
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
 
  delay(TX_INTERVAL - (millis() - start_time)); // wait here for upto TX_INTERVAL ms
}
