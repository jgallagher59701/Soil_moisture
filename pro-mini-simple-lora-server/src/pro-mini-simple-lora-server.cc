/*
  Main node for teh HAST project's leaf node sensor. Based on:
  
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

#include <RH_RF95.h>
#include <SPI.h>
#include <SdFat.h>
#if 0
#include <LiquidCrystal.h>
#endif

#include "data_packet.h"

#define BAUDRATE 9600

#define RFM95_INT 3 // INT1
#define RFM95_CS 5
#define RFM95_RST 6

#define SD_CS 10

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

#define LED 9
#define FREQUENCY 915.0 //902.3

// Singletons for the SD card objects
SdFat sd;    // File system object.
SdFile file; // Log file.

#define FILE_NAME "Sensor_data.csv"

bool sd_card_status = false;    // true == SD card init'd

#if 0
LiquidCrystal lcd(7, 8, 9, A0, A1, A2);
#endif

#define REPLY 0

// Tx should not use the Serial interface except for debugging
#define DEBUG 0

#if DEBUG
#define IO(x) \
    do        \
    {         \
        x;    \
    } while (0)
#else
#define IO(x)
#endif

/**
    @brief RF95 off the SPI bus to enable SD card access
*/
void yield_spi_to_sd()
{
    digitalWrite(SD_CS, LOW);
    digitalWrite(RFM95_CS, HIGH);
}

/**
    @brief RF95 off the SPI bus to enable SD card access
*/
void yield_spi_to_rf95()
{
    digitalWrite(SD_CS, HIGH);
    digitalWrite(RFM95_CS, LOW);
}

/**
   @brief Write a header for the new log file.
   @param file_name open/create this file, append if it exists
   @note Claim the SPI bus
*/
void write_header(const char *file_name)
{
    if (!sd_card_status)
        return;

    yield_spi_to_sd();

    if (!file.open(file_name, O_WRONLY | O_CREAT | O_APPEND))
    {
        IO(Serial.println(F("Couldn't write file header")));
    }

    file.println(F("# Start Log"));
    file.println(F("# Node, Message, Time, Battery V, Last TX Dur ms, Temp C, Hum %, Status"));
    file.close();
}

/**
   @brief log data
   write data to the log, append a new line
   @param file_name open for append
   @param data write this char string
   @note Claim the SPI bus (calls yield_spi_to_sd()().
*/
void log_data(const char *file_name, const char *data)
{
    if (!sd_card_status)
        return;

    yield_spi_to_sd();

    if (file.open(file_name, O_WRONLY | O_CREAT | O_APPEND))
    {
        file.println(data);
        file.close();
    }
    else 
    {
        Serial.print(F("Failed to log data."));
    }
}

void setup()
{
    pinMode(LED, OUTPUT);
    pinMode(RFM95_RST, OUTPUT);
    pinMode(SD_CS, OUTPUT);

    Serial.begin(BAUDRATE);
    Serial.println(F("boot"));

    // Initialize the SD card
    yield_spi_to_sd();

    Serial.print(F("Initializing SD card..."));

    // Initialize at the highest speed supported by the board that is
    // not over 50 MHz. Try a lower speed if SPI errors occur.
    if (sd.begin(SD_CS, SD_SCK_MHZ(50)))
    {
        sd_card_status = true;
    }
    else {
        Serial.println(F("Couldn't init the SD Card"));
        sd_card_status = false;
    }

    // Write data header. This will call error_blink() if it fails.
    write_header(FILE_NAME);

    yield_spi_to_rf95();

    Serial.println(F("Starting receiver"));

    // LORA manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(20);
    digitalWrite(RFM95_RST, HIGH);
    delay(20);

    if (!rf95.init())
    {
        Serial.println(F("Receiver initialization failed"));
    }

    // Setup ISM FREQUENCY
    rf95.setFrequency(FREQUENCY);
    // Setup Power,dBm
    rf95.setTxPower(13);

    // Setup Spreading Factor (6 ~ 12)
    rf95.setSpreadingFactor(7);

    // Setup BandWidth, option: 7800,10400,15600,20800,31200,41700,62500,125000,250000,500000
    rf95.setSignalBandwidth(125000);

    // Setup Coding Rate:5(4/5),6(4/6),7(4/7),8(4/8)
    rf95.setCodingRate4(5);

    Serial.print(F("Listening on frequency: "));
    Serial.println(FREQUENCY);

    Serial.flush();
}

void loop()
{
    yield_spi_to_rf95();

    if (rf95.available())
    {
#if 0
        packet_t buf;
        uint8_t len = sizeof(packet_t);
#else
        uint8_t buf[128];
        uint8_t len = 128;
#endif
        if (rf95.recv((uint8_t*)&buf[0], &len)) {
            digitalWrite(LED, HIGH);

            Serial.print(F("Received length: "));
            Serial.println(len, DEC);
            Serial.flush();

            if (len == sizeof(packet_t)) {

                // Print received packet
                Serial.print(F("Data: "));
                Serial.print(data_packet_to_string((packet_t *)&buf, /* pretty */ true));
                Serial.print(F(", RSSI "));
                Serial.print(rf95.lastRssi(), DEC);
                Serial.print(F(" dBm, SNR "));
                Serial.print(rf95.lastSNR(), DEC);
                Serial.print(F(" dB, good/bad packets: "));
                Serial.print(rf95.rxGood(), DEC);
                Serial.print(F("/"));
                Serial.println(rf95.rxBad(), DEC);

                // log reading to the SD card
                const char *pretty_buf = data_packet_to_string((packet_t *)&buf, false);
                log_data(FILE_NAME, pretty_buf);
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
            }
            // TODO this
            else if (len == 3) { // The  "OK" message after the SD card write
                Serial.print(F("Got: "));
                Serial.println((char*)&buf);
                log_data(FILE_NAME, (char*)&buf);
            }
            else {
                Serial.print(F("Got: "));
                // Add a null to the end of the packet and print as text
                //buf[len] = 0;
                Serial.println((char*)&buf);
                log_data(FILE_NAME, (char*)&buf);
            }
            
            digitalWrite(LED, LOW);
        }
        else
        {
            Serial.println(F("recv failed"));
        }
    }
}
