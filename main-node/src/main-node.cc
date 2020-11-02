/*
  Main node for the HAST project's leaf node sensor.
  6/27/2020
  by jhrg

  Modified to use the ESP8266 'NodeMCU' board since the AT328 does not
  have enough memory for the SD card, LoRa dn DS3231 clock.
  10/24/20
*/

#include <Arduino.h>

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <SdFat.h>

#include <RTClibExtended.h>
#include <Wire.h>

#include "data_packet.h"

#define BUILD_ESP8266_NODEMCU 1

#if BUILD_ESP8266_NODEMCU
#define RFM95_RST D0 // GPIO 16
#define RFM95_INT D2 // GPIO 4
#define RFM95_CS D8  // GPIO 15
#define BAUD_RATE 115200
//#define LED LED_BUILTIN
#elif BUILD_PRO_MINI
#define RFM95_INT 3 // INT1
#define RFM95_CS 5
#define RFM95_RST 6
#define BAUD_RATE 9600
#else
#error "Must define on of BUILD_PRO_MINI or BUILD_ESP8266_NODEMCU"
#endif

#define I2C_SDA D3 // GPIO 0
#define I2C_SCL D1 // GPIO 5

#define SD_CS 10 // SDD3

#define SD 1
#define LORA 1

// Real time clock
RTC_DS3231 RTC; // we are using the DS3231 RTC

#if LORA
#define MAIN_NODE_ADDRESS 0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
// Singleton instance for the reliable datagram manager
RHReliableDatagram rf95_manager(rf95, MAIN_NODE_ADDRESS);
#endif

// #define FREQUENCY 915.0
#define FREQUENCY 902.3
#define SIGNAL_STRENTH 13 // dBm

// Use these settings:
// RH_RF95_SPREADING_FACTOR_1024CPS
// RH_RF95_BW_125KHZ
// RH_RF95_CODING_RATE_4_5
// RH_CAD_DEFAULT_TIMEOUT 10seconds

#if SD
// Singletons for the SD card objects
SdFat sd;    // File system object.
SdFile file; // Log file.
#endif

#define FILE_NAME "Sensor_data.csv"

bool sd_card_status = false; // true == SD card init'd

#define REPLY 1

// Given a DateTime instance, return a pointer to static string that holds
// an ISO 8601 print representation of the object.

char *iso8601_date_time(DateTime t) {
    static char date_time_str[32];

    char val[12];
    date_time_str[0] = '\0';
    strncat(date_time_str, itoa(t.year(), val, 10), sizeof(date_time_str) - 1);
    strncat(date_time_str, "-", sizeof(date_time_str) - 1);
    if (t.month() < 10)
        strncat(date_time_str, "0", sizeof(date_time_str) - 1);
    strncat(date_time_str, itoa(t.month(), val, 10), sizeof(date_time_str) - 1);
    strncat(date_time_str, "-", sizeof(date_time_str) - 1);
    if (t.day() < 10)
        strncat(date_time_str, "0", sizeof(date_time_str) - 1);
    strncat(date_time_str, itoa(t.day(), val, 10), sizeof(date_time_str) - 1);
    strncat(date_time_str, "T", sizeof(date_time_str) - 1);
    if (t.hour() < 10)
        strncat(date_time_str, "0", sizeof(date_time_str) - 1);
    strncat(date_time_str, itoa(t.hour(), val, 10), sizeof(date_time_str) - 1);
    strncat(date_time_str, ":", sizeof(date_time_str) - 1);
    if (t.minute() < 10)
        strncat(date_time_str, "0", sizeof(date_time_str) - 1);
    strncat(date_time_str, itoa(t.minute(), val, 10), sizeof(date_time_str) - 1);
    strncat(date_time_str, ":", sizeof(date_time_str) - 1);
    if (t.second() < 10)
        strncat(date_time_str, "0", sizeof(date_time_str) - 1);
    strncat(date_time_str, itoa(t.second(), val, 10), sizeof(date_time_str) - 1);

    return date_time_str;
}

/**
    @brief RF95 off the SPI bus to enable SD card access
*/
void yield_spi_to_sd() {
    digitalWrite(SD_CS, LOW);
    digitalWrite(RFM95_CS, HIGH);
}

/**
    @brief RF95 off the SPI bus to enable SD card access
*/
void yield_spi_to_rf95() {
    digitalWrite(SD_CS, HIGH);
    digitalWrite(RFM95_CS, LOW);
}

/**
   @brief Write a header for the new log file.
   @param file_name open/create this file, append if it exists
   @note Claim the SPI bus
*/
void write_header(const char *file_name) {
    if (!sd_card_status)
        return;

    yield_spi_to_sd();

    if (!file.open(file_name, O_WRONLY | O_CREAT | O_APPEND)) {
        Serial.println(F("Couldn't write file header"));
        sd_card_status = false;
        return;
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
void log_data(const char *file_name, const char *data) {
    if (!sd_card_status)
        return;

    yield_spi_to_sd();

    if (file.open(file_name, O_WRONLY | O_CREAT | O_APPEND)) {
        file.println(data);
        file.close();
    } else {
        Serial.print(F("Failed to log data."));
    }
}

void status_on() {
    digitalWrite(LED_BUILTIN, LOW);
}

void status_off() {
    digitalWrite(LED_BUILTIN, HIGH);
}

void yield(unsigned long duration_ms) {
    unsigned long start = millis();
    do {
        yield();
    } while (millis() - start < duration_ms);
}

void print_rfm95_info() {
    Serial.print(F("RSSI "));
    Serial.print(rf95.lastRssi(), DEC);
    Serial.print(F(" dBm, SNR "));
    Serial.print(rf95.lastSNR(), DEC);
    Serial.print(F(" dB, good/bad packets: "));
    Serial.print(rf95.rxGood(), DEC);
    Serial.print(F("/"));
    Serial.println(rf95.rxBad(), DEC);
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(RFM95_RST, OUTPUT);
    pinMode(RFM95_CS, OUTPUT); // TODO not need once RFM works
    pinMode(SD_CS, OUTPUT);

    digitalWrite(RFM95_RST, HIGH);

    Serial.begin(BAUD_RATE);
    Serial.println(F("boot"));
    Serial.flush();

    int sda = I2C_SDA;
    int scl = I2C_SCL;
    Wire.begin(sda, scl);

#if SD
    // Initialize the SD card
    yield_spi_to_sd();

    Serial.print(F("Initializing SD card..."));

    // Initialize at the highest speed supported by the board that is
    // not over 50 MHz. Try a lower speed if SPI errors occur.
    if (sd.begin(SD_CS, SD_SCK_MHZ(50))) {
        Serial.println(F(" OK"));
        sd_card_status = true;
    } else {
        Serial.println(F(" Couldn't init the SD Card"));
        sd_card_status = false;
    }

    Serial.flush();
#endif
    // Write data header.
    write_header(FILE_NAME);

#if LORA
    yield_spi_to_rf95();

    Serial.print(F("Starting receiver..."));

    // LORA manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(20);
    digitalWrite(RFM95_RST, HIGH);
    delay(20);

    if (rf95_manager.init()) {
        Serial.println(F(" OK"));

        // Setup ISM FREQUENCY
        rf95.setFrequency(FREQUENCY);
        // Setup Power,dBm
        rf95.setTxPower(SIGNAL_STRENTH);
        // Setup Spreading Factor (CPS == 2^n, N is 6 ~ 12)
        rf95.setSpreadingFactor(RH_RF95_SPREADING_FACTOR_1024CPS);
        // Setup BandWidth, option: 7800,10400,15600,20800,31200,41700,62500,125000,250000,500000
        rf95.setSignalBandwidth(RH_RF95_BW_125KHZ);
        // Setup Coding Rate:5(4/5),6(4/6),7(4/7),8(4/8)
        rf95.setCodingRate4(RH_RF95_CODING_RATE_4_5);
        // Set the CAD timeout to 10s
        rf95.setCADTimeout(RH_CAD_DEFAULT_TIMEOUT);

        Serial.print(F("Listening on frequency: "));
        Serial.println(FREQUENCY);
    } else {
        Serial.println(F(" receiver initialization failed"));
    }
#endif

    Serial.print(F("Current time: "));
    Serial.println(iso8601_date_time(RTC.now()));

    Serial.flush();
}

uint8_t rf95_buf[RH_RF95_MAX_MESSAGE_LEN];

void loop() {
    yield_spi_to_rf95();
#if LORA
    if (rf95.available()) {
        status_on();

        Serial.print(F("Current time: "));
        Serial.println(iso8601_date_time(RTC.now()));

        uint8_t len = sizeof(rf95_buf);
        uint8_t from;

        if (rf95_manager.recvfromAck(rf95_buf, &len, &from)) {

            Serial.print(F("Received length: "));
            Serial.println(len, DEC);
            Serial.flush();

            if (len == sizeof(packet_t)) {

                // Print received packet
                Serial.print(F("Data: "));
                Serial.print(data_packet_to_string((packet_t *)&rf95_buf, /* pretty */ true));

                Serial.print(F(", "));
                print_rfm95_info();
#if 0
                Serial.print(F(", RSSI "));
                Serial.print(rf95.lastRssi(), DEC);
                Serial.print(F(" dBm, SNR "));
                Serial.print(rf95.lastSNR(), DEC);
                Serial.print(F(" dB, good/bad packets: "));
                Serial.print(rf95.rxGood(), DEC);
                Serial.print(F("/"));
                Serial.println(rf95.rxBad(), DEC);
#endif
                // log reading to the SD card
                const char *pretty_buf = data_packet_to_string((packet_t *)&rf95_buf, false);
                log_data(FILE_NAME, pretty_buf);
#if REPLY
                // Send a reply that includes a time code (unixtime)
                uint32_t now = RTC.now().unixtime();
                unsigned long start = millis();
                if (rf95_manager.sendtoWait((uint8_t *)&now, sizeof(now), from)) {
                    // rf95.waitPacketSent();
                    unsigned long end = millis();
                    Serial.print(F("...sent a reply, "));
                    Serial.print(end - start, DEC);
                    Serial.println(F("ms"));
                } else {
                    unsigned long end = millis();
                    Serial.print(F("...reply failed, "));
                    Serial.print(end - start, DEC);
                    Serial.println(F("ms"));
                }
#endif
            } else {
                Serial.print(F("Got: "));
                // Add a null to the end of the packet and print as text
                //rf95_buf[len] = 0;
                Serial.println((char *)&rf95_buf);

                Serial.print(F("RFM95 info: "));
                print_rfm95_info();

                log_data(FILE_NAME, (char *)&rf95_buf);
            }
        } else {
            Serial.println(F("recv failed"));
        }

        status_off();
    }
#endif
}
