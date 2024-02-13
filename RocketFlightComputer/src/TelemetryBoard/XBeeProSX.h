#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <vector>
#include <string>
#include <algorithm>

class XbeeProSX {

public:

    XbeeProSX(uint8_t cs_pin);

    void begin();

    bool isDataAvailable();

    void send(const std::vector<uint8_t> &address, const std::string &message);
    std::pair<std::vector<uint8_t>, std::string> receive();

    void broadcast(const std::string &message);

    void updateSubscribers();
    void sendToSubscribers(const std::string &message);

private:

    uint8_t _cs_pin;

    std::vector<std::vector<uint8_t>> subscribers; // List of subscribers

};