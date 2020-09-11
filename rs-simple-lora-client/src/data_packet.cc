// functions to build, parse and print data packets for the soil
// moisture sensor leaf nodes.

#include <Arduino.h>
#include <assert.h>

#include "data_packet.h"

/**
 * @brief Encode information for transmission
 * Information is stored in the data buffer. The data packets uses 12 bytes.
 * @param data Pointer to a buffer
 * @param node Unsigned Byte the holds the node number
 * @param message Unsigned Long that holds the message number
 * @param time Unsigned Long that holds the epoch time (seconds since 1/1/1970)
 * @param battery Unsigend Short with battery voltage * 100
 * @param last_tx_duration Unsigned Short ms duration of the previous transmission
 * @param temp Short with temperature in degrees C * 100
 * @param humidity Unsigend Short with percent relative humidity * 100
 * @param status Unsigned Byte with the leaf node status.
 * @return The number of bytes in the packet.
 */
void build_data_packet(packet_t *data, const uint8_t node, const uint32_t message,
                       const uint32_t time, const uint16_t battery, const uint16_t last_tx_duration,
                       const int16_t temp, const uint16_t humidity, const uint8_t status) {

    SerialUSB.println("Start of build data packet");

    data->node = node;
    data->message = message;
    data->time = time;
    data->battery = battery;
    data->last_tx_duration = last_tx_duration;
    data->temp = temp;
    data->humidity = humidity;
    data->status = status;
}

/**
 * @brief Unpack a data packet
 * @param data Pointer to a DATA_PACKET_SIZE byte data packet
 * @param node Value-result parameter for the node number. if NULL, no value is extracted
 * @param message if not NULL, V-R parameter for the message number
 * @param time If not NULL, V-R parameter for epoch time. 
 * @param battery If not NULL, V-R parameter for battery voltage *100
 * @param last_tx_duration if not NULL, V-R parameter for the last tx duration in ms
 * @param temp If not NULL, V-R parameter for temperature om C * 100
 * @param humidity If not NULL, V-R parameter for percent rel. humidity * 100
 * @param status If not NULL, V-R parameter for status info
 */
void parse_data_packet(const packet_t *data, uint8_t *node, uint32_t *message, uint32_t *time,
                       uint16_t *battery, uint16_t *last_tx_duration, int16_t *temp, uint16_t *humidity, uint8_t *status) {
   if (node)
        *node = data->node;

    if (message)
        *message = data->message;

    if (time)
        *time = data->time;

    if (battery)
        *battery = data->battery;

    if (last_tx_duration)
        *last_tx_duration = data->last_tx_duration;

    if (temp)
        *temp = data->temp;

    if (humidity)
        *humidity = data->humidity;

    if (status)
        *status = data->status;
}

/**
 * @brief print the data packet to a string
 * 
 * Print the data in packet to a string. The returned pointer is to
 * static storage and will be changed by subsequent calls to this funtion.
 * If the optional parameter \c pretty is true, add info for a human.
 * 
 * @param data The data packet
 * @param pretty Optional, if true, print names, units, etc. Default: false
 * @return Pointer to the string in static storage.
 */
char *data_packet_to_string(const packet_t *data, bool pretty /* false */) {
    uint8_t node;
    uint32_t time;
    uint32_t message;
    uint16_t battery;
    uint16_t last_tx_duration;
    int16_t temp;
    uint16_t humidity;
    uint8_t status;

    parse_data_packet(data, &node, &message, &time, &battery, &last_tx_duration, &temp, &humidity, &status);

    static char decoded_string[128];
    if (pretty) {
        // string length 62 characters + 2 bytes (6 chars) + 2 Longs (20) + 3 Shorts (15)
        // = 62 + 41 = 103
        snprintf((char *)decoded_string, sizeof(decoded_string),
                 "node: %d, message: %d, time: %d, Vbat %d v, Tx dur %d ms, T: %d C, RH: %d %%, status: 0x%02x",
                 node, message, time, battery, last_tx_duration, temp, humidity, (unsigned int)status);
    } else {
        snprintf((char *)decoded_string, sizeof(decoded_string),
                 "%d, %d, %d, %d, %d, %d, %d, 0x%02x",
                 node, message, time, battery, last_tx_duration, temp, humidity, (unsigned int)status);
    }

    return decoded_string;
}
