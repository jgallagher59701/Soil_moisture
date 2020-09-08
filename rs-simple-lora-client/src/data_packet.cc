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
 * @param temp Short with temperature in degrees C * 100
 * @param humidity Unsigend Short with percent relative humidity * 100
 * @param status Unsigned Byte with the leaf node status.
 * @return The number of bytes in the packet.
 */
uint32_t build_data_packet(uint8_t *data, const uint8_t node, const uint32_t message,
                           const uint32_t time, const uint16_t battery,
                           const int16_t temp, const uint16_t humidity, const uint8_t status) {
    uint8_t *ldata = (uint8_t *)memcpy(data, (const void *)node, sizeof(node));
    ldata += sizeof(node);

    memcpy(ldata, (const void *)message, sizeof(message));
    ldata += sizeof(message);

    memcpy(ldata, (const void *)time, sizeof(time));
    ldata += sizeof(time);

    memcpy(ldata, (const void *)battery, sizeof(battery));
    ldata += sizeof(battery);

    memcpy(ldata, (const void *)temp, sizeof(temp));
    ldata += sizeof(temp);

    memcpy(ldata, (const void *)humidity, sizeof(humidity));
    ldata += sizeof(humidity);

    memcpy(ldata, (const void *)status, sizeof(status));
    ldata += sizeof(status);

    assert(ldata - data == DATA_PACKET_SIZE);

    return DATA_PACKET_SIZE;
}

/**
 * @brief Unpack a data packet
 * @param data Pointer to a DATA_PACKET_SIZE byte data packet
 * @param node Value-result parameter for the node number. if NULL, no value is extracted
 * @param message if not NULL, V-R parameter for the message number
 * @param time If not NULL, V-R parameter for epoch time. 
 * @param battery If not NULL, V-R parameter for battery voltage *100
 * @param temp If not NULL, V-R parameter for temperature om C * 100
 * @param humidity If not NULL, V-R parameter for percent rel. humidity * 100
 * @param status If not NULL, V-R parameter for status info
 */
void parse_data_packet(const uint8_t *data, uint8_t *node, uint32_t *message,
                       uint32_t *time, uint16_t *battery, int16_t *temp, uint16_t *humidity,
                       uint8_t *status) {
    if (node)
        memcpy((void *)node, (const void *)data, sizeof(uint8_t));
    data += sizeof(uint8_t);

    if (message)
        memcpy((void *)message, (const void *)data, sizeof(uint32_t));
    data += sizeof(uint32_t);

    if (time)
        memcpy((void *)time, (const void *)data, sizeof(uint32_t));
    data += sizeof(uint32_t);

    if (battery)
        memcpy((void *)battery, (const void *)data, sizeof(uint16_t));
    data += sizeof(uint16_t);

    if (temp)
        memcpy((void *)temp, (const void *)data, sizeof(int16_t));
    data += sizeof(int16_t);

    if (humidity)
        memcpy((void *)humidity, (const void *)data, sizeof(uint16_t));
    data += sizeof(uint16_t);

    if (status)
        memcpy((void *)status, (const void *)data, sizeof(uint8_t));
}

/**
 * @brief print the data packet to a string
 * If the optional parameter \c pretty is true, add info for a human.
 * @param data The data packet
 * @param pretty Optional, if true, print names, units, etc. Default: false
 * @return Pointer to the string in static storage.
 */
char *data_packet_to_string(const uint8_t *data, bool pretty /* false */) {
    uint8_t *node;
    uint32_t *time;
    uint32_t *message;
    uint16_t *battery;
    int16_t *temp;
    uint16_t *humidity;
    uint8_t *status;

    parse_data_packet(data, node, message, time, battery, temp, humidity, status);

    static char decoded_string[128];
    if (pretty) {
        // string length 62 characters + 2 bytes (6 chars) + 2 Longs (20) + 3 Shorts (15)
        // = 62 + 41 = 103
        snprintf((char *)decoded_string, sizeof(decoded_string),
                 "node: %d, message: %d, time: %d, Vbat %d v, T: %d C, RH: %d %%, status: 0x%02x",
                 node, message, time, battery, temp, humidity, status);
    } else {
        snprintf((char *)decoded_string, sizeof(decoded_string),
                 "%d, %d, %d, %d, %d, %d, 0x%02x",
                 node, message, time, battery, temp, humidity, status);
    }
}
