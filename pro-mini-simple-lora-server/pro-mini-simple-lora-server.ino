/*
  LoRa Simple Yun Server :
  
  Example sketch showing how to create a simple messageing server, 
  with the RH_RF95 class. RH_RF95 class does not provide for addressing or
  reliability, so you should only use RH_RF95 if you do not need the higher
  level messaging abilities.

  It is designed to work with the other example LoRa Simple Client

  User need to use the modified RadioHead library from:
  https://github.com/dragino/RadioHead

  modified 6/27/2020
  by jhrg
*/


#include <SPI.h>
#include <RH_RF95.h>

#define BAUDRATE 9600

#define RFM95_INT 3 // INT1
#define RFM95_CS 5
#define RFM95_RST 6

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

int led = 9;
float frequency = 915.0;

#define REPLY 0

// Tx should not use the Serial interface except for debugging
#define DEBUG 0

#if DEBUG
#define IO(x) do { x; } while (0)
#else
#define IO(x)
#endif

void setup() 
{
  pinMode(led, OUTPUT); 
  pinMode(RFM95_RST, OUTPUT);
  
  while (!Serial);
  Serial.begin(BAUDRATE);
  Serial.println(F("boot"));
  
  Serial.println(F("Start receiveer"));

  // LORA manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(20);
  digitalWrite(RFM95_RST, HIGH);
  delay(20);
    
  if (!rf95.init()) {
    Serial.println(F("init failed"));
    while(true);
  }
  
  // Setup ISM frequency
  rf95.setFrequency(frequency);
  // Setup Power,dBm
  rf95.setTxPower(13);
  
  // Setup Spreading Factor (6 ~ 12)
  rf95.setSpreadingFactor(7);
  
  // Setup BandWidth, option: 7800,10400,15600,20800,31200,41700,62500,125000,250000,500000
  rf95.setSignalBandwidth(125000);
  
  // Setup Coding Rate:5(4/5),6(4/6),7(4/7),8(4/8) 
  rf95.setCodingRate4(5);
  
  Serial.print(F("Listening on frequency: "));
  Serial.println(frequency);

#if 0
  // Set date and time using compiler constants.
  DateTime(F(__DATE__), F(__TIME__))
  DateTime(2014, 1, 21, 3, 0, 0)
#endif
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
      digitalWrite(led, HIGH);
      RH_RF95::printBuffer("request: ", buf, len);

     // Print received datagram as CSV
      Serial.print((char*)buf);
      Serial.print(", RSSI ");
      Serial.print(rf95.lastRssi(), DEC);
      Serial.print(" dBm, SNR ");
      Serial.print(rf95.lastSNR(), DEC);
      Serial.print(" dB, good/bad packets: ");
      Serial.print(rf95.rxGood(), DEC);
      Serial.print("/");
      Serial.println(rf95.rxBad(), DEC);
      
#if REPLY
      // Send a reply
      uint8_t data[] = "And hello back to you";
      unsigned long start = millis();
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      unsigned long end = millis();
      IO(Serial.print(F("...sent a reply, ")));
      IO(Serial.print(end - start, DEC));
      IO(Serial.println(F("ms")));
#endif

      digitalWrite(led, LOW);
    }
    else
    {
      Serial.println(F("recv failed"));
    }
  }
}
