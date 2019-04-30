
// -*- mode: C++ -*-
//
// Soil Moisture sensor
//
// Transmit information to a second LoRa radio unit. 

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_INT 3
#define RFM95_RST 6
#define RFM95_EN 7
#define RFM95_CS 10

#define LORA_INIT_STATUS 4

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Tx should not use the Serial interface
#define DEBUG 0
#if DEBUG
#define IO(x) do { (x); } while (0)
#else
#define IO(x)
#endif

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

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
  IO(Serial.println("Arduino LoRa TX Test!"));

  // LED to show the LORA radio has been configured.
  pinMode(LORA_INIT_STATUS, OUTPUT);
  digitalWrite(LORA_INIT_STATUS, LOW);

  // LORA manual reset
  pinMode(RFM95_RST, OUTPUT);
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
    blink_forever(LORA_INIT_STATUS, 10 /* Hz */); // 10 Hz blink
  }
  IO(Serial.println("LoRa radio init OK!"));

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM.
  // If frequenct set fails, blink at 2Hz forever.
  if (!rf95.setFrequency(RF95_FREQ)) {
    IO(Serial.println("setFrequency failed"));
    blink_forever(LORA_INIT_STATUS, 2); // 2 Hz blink
  }
  IO(Serial.print("Set Freq to: "); Serial.println(RF95_FREQ));

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

uint16_t packetnum = 0;   // packet counter, we increment per xmission
int16_t rssi = 0;         // rssi and snr are read for the ACK received in response to a Tx
int snr = 0;

// Send basic information about the sensor plus the information in \arg data.
// Wait for a response and record the SNR and RSSI of the response (used in the
// next send_packet() transmission.
void send_packet(String data)
{
  IO(Serial.println("Sending to rf95_server"));
  
  // Send a message to rf95_server  
  // The RSSI and SNR are for the most recent ACK received from the server in response to 
  // the client's message. The message is 'Hello World#<packetnum>,<Vcc>,<rssi>,<snr>'. 
  // Vcc is X100 (seegetBandgap()).
  String message = String("Hello World: #") + String(++packetnum) + String(',') ;
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

// Blink the LED at pin for freq Hz forever. Used to indicate an error during 
// startup/initialization.
void blink_forever(int pin, int freq)
{
  uint16_t duration = 1000/freq;
  while(1) {
    digitalWrite(pin, LOW);
    delay(duration);
    digitalWrite(pin, HIGH);
    delay(duration);
  }
}

// Set up the radio, halt here if init fails; LEDS provide radio status.
// If one LED lights, that indicates a failure. If both light, that indicates
// Success
void setup() 
{
  IO(while (!Serial));

  IO(Serial.begin(9600));
  IO(delay(100));

  lora_setup();
}

String data_msg = "Clock Temp: ";
void loop()
{ 
  send_packet(data_msg + String(0.0));
  delay(1000);
}
