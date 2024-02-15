#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <vector>
#include <string>
#include <algorithm>

struct ReceivePacket {
    size_t length;
    uint64_t address;
    uint8_t *data;
};

class XbeeProSX {

public:

    XbeeProSX(uint8_t cs_pin);

    void begin();

    bool isDataAvailable();

    void send(uint64_t address, const void *data, size_t size_bytes);
    ReceivePacket* receive();

    void broadcast(const void *data, size_t size);

    void updateSubscribers();
    void sendToSubscribers(const void *data, size_t size);

private:

    uint8_t _cs_pin;

    uint64_t *subscribers;

    uint num_subscribers;

    void add_subscriber(uint64_t address);

};