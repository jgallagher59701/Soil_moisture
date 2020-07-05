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

#define NODE 1

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

float frequency = 915.0;

void setup() 
{
  Serial.begin(9600);
  //while (!Serial) ; // Wait for serial port to be available

  // LORA manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(15);
  // LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  digitalWrite(RFM95_RST, HIGH);
  delay(15);
  //LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);

  Serial.println("Start LoRa Client");
  
  if (!rf95.init()) {
    Serial.println("init failed");
    while (true) ;
  }
  
  // Setup ISM frequency
  rf95.setFrequency(frequency);
  // Setup Power,dBm
  rf95.setTxPower(13);

  // Setup Spreading Factor (6 ~ 12)
  rf95.setSpreadingFactor(7);
  
  // Setup BandWidth, option: 7800,10400,15600,20800,31200,41700,62500,125000,250000,500000
  //Lower BandWidth for longer distance.
  rf95.setSignalBandwidth(125000);
  
  // Setup Coding Rate:5(4/5),6(4/6),7(4/7),8(4/8) 
  rf95.setCodingRate4(5);
}

unsigned long last_tx_time = 0;

void loop()
{
  Serial.println("Sending to LoRa Server");
  // Send a message to LoRa Server
  
  unsigned long start_time = millis();
  
  uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
  snprintf(data, sizeof(data), "Hello, this is device %d, tx time %ld ms", NODE, last_tx_time);
  rf95.send(data, sizeof(data));
  
  rf95.waitPacketSent();
  
  unsigned long end_time = millis();
  unsigned long last_tx_time = end_time = start_time;
  
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(3000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("recv failed");
    }
  }
  else
  {
    Serial.println("No reply, is LoRa server running?");
  }
  delay(5000);
}
