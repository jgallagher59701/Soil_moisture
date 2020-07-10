//
// Simple lora server. Uses Simple lora client.
//
// Test the affectiveness of SemTech/HopeRF Channel Activity Detection
// to reduce collisions in a simple broadcast transmission protocol.
//
// Based on LoRa Simple Yun Server by Edwin Chen <support@dragino.com>,
// Dragino Technology Co., Limited
//
// James Gallagher <jgallagher@opendap.org>
// 7/6/20

#define BAUDRATE 115200

#include <Console.h>
#include <SPI.h>
#include <RH_RF95.h>

#define FREQ 915.0
#define BANDWIDTH 125000
#define SPREADING_FACTOR 7  // sf = 6 - 12 --> 2^(sf)
#define CODING_RATE 5

#define CAD_TIMEOUT 3000 // timeout for CAD wait; zero dsables. Only affects replies

// Tx should not use the Serial interface except for debugging
#define DEBUG 1
#define REPLY 0
#define LED A2

#if DEBUG
#define IO(x) do { x; } while (0)
#else
#define IO(x)
#endif

// Singleton instance of the radio driver
RH_RF95 rf95;

unsigned int tx_power = 13;   // dBm 5 tp 23 for RF95

void setup() 
{
  pinMode(LED, OUTPUT);    
   
  Bridge.begin(BAUDRATE);
  Console.begin();

  
  while (!Console) ; // Wait for console port to be available
  
  Console.println(F("Start receiveer"));
  if (!rf95.init()) {
    Console.println(F("init failed"));
    while(true) ;
  }
    
  if (!rf95.init()) {
    Console.println(F("LoRa init failed."));
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

  // Set up CAD for send() calls. Zero disables CAD. Only affects replies
  rf95.setCADTimeout(CAD_TIMEOUT); 
  
  Console.print(F("Listening on frequency: "));
  Console.println(FREQ);
}

void loop()
{
  if (rf95.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len))
    {
      digitalWrite(LED, HIGH);
      RH_RF95::printBuffer("request: ", buf, len);

      // Print received datagram as CSV
      Console.print((char*)buf);
      Console.print(", RSSI ");
      Console.print(rf95.lastRssi(), DEC);
      Console.print(" dBm, SNR ");
      Console.print(rf95.lastSNR(), DEC);
      Console.print(" dB, good/bad packets: ");
      Console.print(rf95.rxGood(), DEC);
      Console.print("/");
      Console.println(rf95.rxBad(), DEC);
      
#if REPLY
      // Send a reply
      uint8_t data[] = "And hello back to you";
      unsigned long start = millis();
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      unsigned long end = millis();
      IO(Console.print(F("...sent a reply, ")));
      IO(Console.print(end - start, DEC));
      IO(Console.println(F("ms")));
#endif

      digitalWrite(LED, LOW);
    }
    else
    {
      Console.println(F("recv failed"));
    }
  }
}
