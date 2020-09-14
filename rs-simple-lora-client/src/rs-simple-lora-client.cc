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

#include <Arduino.h>

#include <string.h>
#include <time.h>

#include <SPI.h>
#include <Wire.h>

#include <RH_RF95.h>
#include <RTCZero.h>
#include <SerialFlash.h>

#include "Adafruit_SHT31.h" // Uses the BusIO library from Adafruit
#include "SdFat.h"

#include "blink.h"
#include "data_packet.h"

#define DEBUG 0          // Requires USB
#define Serial SerialUSB // Needed for RS. jhrg 7/26/20

#include "debug.h"

// Pin assignments

#define RFM95_INT 2 // RF95 Interrupt
#define RFM95_CS 5  // RF95 SPI CS

#define SD_PWR 11 // HIGH == power on SD card
#define SD_CS 10  // CS for the SD card, SPI uses dedicated lines

#define FLASH_CS 4 // CS for 2MB onboard flash on the SPI bus

#define USE_STANDBY 8 // If pin 8 is LOW, use rtc.standbyMode() for delay. Pull HIGH to use yield()
#define MAX_POWER 9   // Not used
#define STATUS_LED 13 // Only for DEBUG mode

#define V_BAT A0

// Constants

// Channel 0 is 902.3, others are + 200KHz for BW = 125 KHz. There are 64 channels.
// 915.0 MHz is the no-channel nominal freq
#define FREQ 902.3
#define BANDWIDTH 125000
#define SPREADING_FACTOR 12 // sf = 6 - 12 --> 2^(sf)
#define CODING_RATE 5

#define NODE 1
#define EXPECT_REPLY 0
#define USE_RTC_STANDBY_FOR_DELAY 1 // 0 uses yield()

#define WAIT_AVAILABLE 3000      // ms to wait for a response from server
#define CAD_TIMEOUT 3000         // ms timeout for CAD wait
#define FAST_TX_INTERVAL_MS 5000 // ms to wait before next transmission when in non-sleeping mode
#define STANDBY_INTERVAL_S 20    // seconds to wait before next transmission when sleeping

#define ADC_BITS 12
#define ADC_MAX_VALUE 4096

// Log file name.
#define FILE_BASE_NAME "Data"

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

unsigned int tx_power = 13; // dBm 5 tp 23 for RF95

// Singleton for the Real Time Clock
RTCZero rtc;

// Temp/humidity sensor. Use the SHT31 code. We actually have the 30-D sensor.
Adafruit_SHT31 sht30d = Adafruit_SHT31();

// Singletons for the SD card objects
SdFat sd;    // File system object.
SdFile file; // Log file.

// setup() error codes. Any of these error the boot of the node
// and flash the status led 2, 3, ..., n times.
#define SHT31_BEGIN_FAIL 2
#define SD_BEGIN_FAIL 3
#define SD_WRITE_HEADER_FAIL 4
#define RFM95_INIT_FAIL 5
#define RFM95_SET_FREQ_FAIL 6
#define SD_LOG_FILE_NAME_FAIL 7

#define STATUS_OK 0x00

#define SD_NO_MORE_NAMES 0x01 // means it will use "Data99.csv"
#define SD_FILE_ENTRY_WRITE_ERROR 0x02
#define SD_CARD_WAKEUP_ERROR 0x04

#define RFM95_NO_REPLY 0x10
#define RFM95_SEND_QUEUE_ERROR 0x20
#define RFM95_SEND_ERROR 0x40
#define RFM95_RECEIVE_FAILED 0x80

#if DEBUG
#define SHT30D 0
#else
#define SHT30D 1
#endif

uint8_t status = STATUS_OK;

/**
   @brief delay that enables background tasks
*/
void yield(unsigned long ms_delay) {
    unsigned long start = millis();
    while ((millis() - start) < ms_delay)
        yield();
}

/**
    @brief RF95 off the SPI bus to enable SD card access
*/
void yield_spi_to_sd() {
    digitalWrite(RFM95_CS, HIGH);
    // TODO remove? digitalWrite(FLASH_CS, HIGH);
}

/**
    @brief RF95 off the SPI bus to enable SD card access
*/
void yield_spi_to_rf95() {
    digitalWrite(SD_CS, HIGH);
    // TODO remove?  digitalWrite(FLASH_CS, HIGH);
}

/**
   @brief Get the current epoch from __DATE__ and __TIME__
   This function returns the time i seconds since 1 Jan 1970
   using the string values of the compile-time constants
   __DATE__ and __TIME_. The formats of these are: mmm dd yyyy
   (e.g. "Jan 14 2012") and hh::mm::ss in 24 hour time
   (e.g. "22:29:12")
   @note input must be formatted correctly
   @param data The value of __DATE__ or the equiv
   @param time The value of __TIME__ or the equiv
   @return Seconds since Jan 1, 1970
*/
time_t
get_epoch(const char *date, const char *time) {
    char s_month[5];
    struct tm t = {0};
    static const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";

    sscanf(date, "%s %d %d", s_month, &t.tm_mday, &t.tm_year);

    // pointer math
    int month = (strstr(month_names, s_month) - month_names) / 3;

    t.tm_mon = month;
    t.tm_year -= 1900;
    t.tm_isdst = -1;

    sscanf(time, "%d:%d:%d", &t.tm_hour, &t.tm_min, &t.tm_sec);

    return mktime(&t);
}

/**
   @note this version assumes that a voltage divider reduces Vbat by 1/4.3
   @return The battery voltage x 100 as an int
*/
int get_bat_v() {
    // voltage divider v_bat = 4.3 * vadc
    // vadc = (raw / 4096)
    int raw = analogRead(V_BAT);
    return 430 * (raw / (float)ADC_MAX_VALUE); // voltage * 100
}

char file_name[13] = FILE_BASE_NAME "00.csv";

/**
   @brief get an unused filename for the new log.
   This function returns a pointer to local static storage.
   @return A pointer to the new file name.
*/
const char *
get_new_log_filename() {
    const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;

    // Find an unused file name.
    if (BASE_NAME_SIZE > 6) {
        error_blink(STATUS_LED, SD_LOG_FILE_NAME_FAIL);
    }

    // Look for a BASE_NAME00.csv. if all are taken return BASE_NAME99.csv
    while (sd.exists(file_name)) {
        if (file_name[BASE_NAME_SIZE + 1] != '9') {
            file_name[BASE_NAME_SIZE + 1]++;
        } else if (file_name[BASE_NAME_SIZE] != '9') {
            file_name[BASE_NAME_SIZE + 1] = '0';
            file_name[BASE_NAME_SIZE]++;
        } else {
            status |= SD_NO_MORE_NAMES;
            break;
        }
    }

    return file_name;
}

/**
   @return The log file name
*/
const char *
get_log_filename() {
    return file_name;
}

/**
   @brief Write a header for the new log file.
   @param file_name open/create this file, append if it exists
   @note Claim the SPI bus
*/
void write_header(const char *file_name) {
    yield_spi_to_sd();

    if (!file.open(file_name, O_WRONLY | O_CREAT | O_APPEND)) {
        IO(Serial.println(F("Couldn't write file header")));
        error_blink(STATUS_LED, SD_WRITE_HEADER_FAIL);
    }

    file.println(F("# Start Log"));
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
    yield_spi_to_sd();

    SerialUSB.print("data to SD card: ");
    SerialUSB.println(data);

    if (file.open(file_name, O_WRONLY | O_CREAT | O_APPEND)) {
        file.println(data);
        file.close();
    } else {
        status |= SD_FILE_ENTRY_WRITE_ERROR;
    }
}

/**
 * @brief Get the temperature from the SHT-3
 * @return the temperature * 100 as a 16-bit unsigned int
 */
uint16_t get_temperature() {
#if SHT30D
    return (uint16_t)(sht30d.readTemperature() * 100);
#else
    return 0;
#endif
}

/**
 * @brief Get the humidity from the SHT-3
 * @return the humidity * 100 as a 16-bit unsigned int
 */
uint16_t get_humidity() {
#if SHT30D
    return (uint16_t)(sht30d.readHumidity() * 100);
#else
    return 0;
#endif
}

/** 
 * @brief Call back for the sleep alarm
 */
void alarmMatch() {
    // Need to do this?
    rtc.detachInterrupt();
}

void setup() {
    // Blanket pin mode settings
    // Switch unused pins as input and enabled built-in pullup
    for (unsigned int pinNumber = 0; pinNumber < 23; pinNumber++) {
        pinMode(pinNumber, INPUT_PULLUP);
    }

    for (unsigned int pinNumber = 32; pinNumber < 42; pinNumber++) {
        pinMode(pinNumber, INPUT_PULLUP);
    }

    pinMode(25, INPUT_PULLUP);
    pinMode(26, INPUT_PULLUP);

    // pin mode setting for I/O pins used by this code
    pinMode(USE_STANDBY, INPUT_PULLUP);
    pinMode(MAX_POWER, INPUT_PULLUP); // not used

    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, HIGH);

    analogReadResolution(ADC_BITS);

    // RocketScream's built-in flash not used
    SerialFlash.begin(FLASH_CS);
    SerialFlash.sleep();

    // SD card power control (low-side switching)
    pinMode(SD_PWR, OUTPUT);
    digitalWrite(SD_PWR, HIGH); // Power on the card

    // SPI bus control
    pinMode(SD_CS, OUTPUT);
    pinMode(RFM95_CS, OUTPUT);

    // Initialize USB and attach to host. Needed because code will detach for sleep
    USBDevice.init();
    USBDevice.attach();

    // Only start the Serial interface when DEBUG is 1
    IO(Serial.begin(9600));
    IO(while (!Serial)); // Wait for serial port to be available

    IO(Serial.println(F("Start LoRa Client")));

    // Initialize the RTC

    rtc.begin(/*reset*/ true);
    rtc.setEpoch(get_epoch(__DATE__, __TIME__));

    IO(Serial.print(F("Date, Time: ")));
    IO(Serial.print(__DATE__));
    IO(Serial.print(F(", ")));
    IO(Serial.println(__TIME__));
    char date_str[32] = {0};
    snprintf(date_str, sizeof(date_str), "%d/%d/%dT%d:%d:%d", rtc.getMonth(), rtc.getDay(), rtc.getYear(),
             rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
    IO(Serial.print(F("RTC: ")));
    IO(Serial.println((const char *)date_str));

    // Initialize the temp/humidity sensor
#if SHT30D
    if (!sht30d.begin(0x44)) { // Set to 0x45 for alternate i2c addr
        IO(Serial.println(F("Couldn't find SHT31")));
        error_blink(STATUS_LED, SHT31_BEGIN_FAIL);
    }

    // The SHT30-D temp/humidity sensor has a heater; turned it off
    sht30d.heater(false);
#endif // SHT30D

    // Initialize the SD card
    yield_spi_to_sd();

    IO(Serial.print(F("Initializing SD card...")));

    // Initialize at the highest speed supported by the board that is
    // not over 50 MHz. Try a lower speed if SPI errors occur.
    if (!sd.begin(SD_CS, SD_SCK_MHZ(50))) {
        IO(Serial.println(F("Couldn't init the SD Card")));
        error_blink(STATUS_LED, SD_BEGIN_FAIL);
    }

    const char *file_name = get_new_log_filename();
    IO(Serial.println(file_name));

    // Write data header. This will call error_blink() if it fails.
    write_header(file_name);

    yield_spi_to_rf95();

    if (!rf95.init()) {
        IO(Serial.println(F("LoRa init failed.")));
        error_blink(STATUS_LED, RFM95_INIT_FAIL);
    }

    // Setup ISM frequency
    if (!rf95.setFrequency(FREQ)) {
        IO(Serial.println(F("LoRa frequency out of range.")));
        error_blink(STATUS_LED, RFM95_SET_FREQ_FAIL);
    }

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

    // Because the RS Ultra Pro boards native USB won't work with the standby() mode
    // in the LowPower or RTCZero libraries, the MCU board can easily wind up bricked
    // when using standby(). It will then become impossible to upload new/fixed
    // code. Add a 10s delay here so a coordinated reset/upload will work. This is
    // not strictly needed for this code - the short delay loop option has the USB
    // attached, but this is convenient because the node's mode does not have to be
    // changed.
    yield(10000);
}

void loop() {
    // FIXME - grab the time here and then use that time plus an offset (STANDBY_INTERVAL_S)
    // to determine when to wake up so that the node will wake up on the hour, etc.
    // Set the initial time so some sensible value.
    IO(Serial.print(F("Sending to LoRa Server.. ")));
    static unsigned long last_tx_time = 0;
    static unsigned long message = 0;

    // Send a message to LoRa Server

    ++message;

    digitalWrite(STATUS_LED, HIGH); // TODO only in DEBUG mode

    yield_spi_to_rf95();

#if 0
    uint8_t data[DATA_PACKET_SIZE];
    snprintf((char *)data, sizeof(data),
             "Hello, node %d, message %ld, msg time %ld, tx time %ld ms, battery %d v, temp %d C, humidity %d %%, status 0x%02x",
             NODE, message, rtc.getEpoch(), last_tx_time, get_bat_v(), get_temperature(), get_humidity(), status);
#endif

    // New packet encoding
    packet_t data;
    build_data_packet(&data, NODE, message, rtc.getEpoch(), get_bat_v(), (uint16_t)last_tx_time,
                      get_temperature(), get_humidity(), status);

    IO(Serial.println(data_packet_to_string(&data, true)));

    unsigned long start_time_ms = millis();
    status = STATUS_OK; // clear status for the next sample interval

    // This may block for up to CAD_TIMEOUT s
    if (!rf95.send((const uint8_t *)&data, DATA_PACKET_SIZE)) {
        status |= RFM95_SEND_QUEUE_ERROR;
    }
    // Block until packet sent. The code could run the SD card and radio in parallel,
    // but the potential current draw could strain the batteries. Wait for the radio
    // to finish, then write to the SD card.
    if (!rf95.waitPacketSent()) {
        status |= RFM95_SEND_ERROR;
    }

    unsigned long end_time = millis();
    last_tx_time = end_time - start_time_ms; // last_tx_time used next iteration

#if EXPECT_REPLY
    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.waitAvailableTimeout(WAIT_AVAILABLE)) {
        // Should be a reply message for us now
        if (rf95.recv(buf, &len)) {
            IO(Serial.print(F("got reply: ")));
            IO(Serial.println((char *)buf));
            IO(Serial.print(F("RSSI: ")));
            IO(Serial.println(rf95.lastRssi(), DEC));
        } else {
            IO(Serial.println(F("receive failed")));
            status |= RFM95_RECEIVE_FAILED;
        }
    } else {
        IO(Serial.println(F("No reply, is LoRa server running?")));
        status |= RFM95_NO_REPLY;
    }
#endif // EXPECT_REPLY

    log_data(get_log_filename(), data_packet_to_string(&data));

    digitalWrite(STATUS_LED, LOW);

    // Leaving this in guards against bricking the RS when sleeping with the USB detached.
    if (digitalRead(USE_STANDBY) == LOW) {
        // low-power configuration
        USBDevice.detach();
        rf95.sleep();              // Turn off the LoRa
        digitalWrite(SD_PWR, LOW); // Turn off the SD card
        // Adding SPI.end() drops the measured current draw from 0.65mA to 0.27mA
        SPI.end();

        uint8_t offset = max(STANDBY_INTERVAL_S - max((millis() - start_time_ms) / 1000, 0), 0);
        IO(Serial.print(F("Alarm offset: ")));
        IO(Serial.println(offset));
        rtc.setAlarmEpoch(rtc.getEpoch() + offset);
        rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
        rtc.attachInterrupt(alarmMatch);
        rtc.standbyMode();

        // Reverse low-power options
        SPI.begin();
        digitalWrite(SD_PWR, HIGH);
        // rf95 wakes up on the first function call.
        if (!sd.begin(SD_CS, SD_SCK_MHZ(50))) {
            status |= SD_CARD_WAKEUP_ERROR;
        }
        // wait 250ms for the SD card. Maybe this is not needed in most iterations because
        // the radio blocks for a while, but in some cases it might be needed.
        yield(250);
    } else {
        unsigned long elapsed_time_ms = max((millis() - start_time_ms), 0);
        yield(max(FAST_TX_INTERVAL_MS - elapsed_time_ms, 0)); // wait here for upto TX_INTERVAL seconds
    }
}
