// Header for the data_packet functions

#ifndef h_data_packet_h
#define h_data_packet_h

#include <Arduino.h>

/// Size of the data packet in bytes
#define DATA_PACKET_SIZE sizeof(packet_t)

struct packet_t {
    uint8_t data;
    uint8_t node;
    uint32_t message;
    uint32_t time;
    uint16_t battery;
    uint16_t last_tx_duration;
    int16_t temp;
    uint16_t humidity;
    uint8_t status;
};

void build_data_packet(packet_t *data, const uint8_t node, const uint32_t message, const uint32_t time,
                  const uint16_t battery, const uint16_t last_tx_duration,
                  const int16_t temp, const uint16_t humidity, const uint8_t status);

void parse_data_packet(const packet_t *data, uint8_t *node, uint32_t *message, uint32_t *time, uint16_t *battery,
                       uint16_t *last_tx_duration, int16_t *temp, uint16_t *humidity, uint8_t *status);

char *data_packet_to_string(const packet_t *data, bool pretty = false);

#endif