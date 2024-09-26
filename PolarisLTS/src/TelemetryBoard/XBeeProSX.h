#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "xbee/XBeeDevice.h"

class XbeeProSX: public XBeeDevice {
public:
    XbeeProSX(uint8_t cs_pin);

    void writeBytes(const char *data, size_t length_bytes) override;

    void handleReceivePacket(XBee::ReceivePacket::Struct *frame) override;

    void handleReceivePacket64Bit(XBee::ReceivePacket64Bit::Struct *frame) override;

    void didCycle() override;

    void start() override;

    void incorrectChecksum(uint8_t calculated, uint8_t received) override;

    void log(const char *format, ...) override;

private:
    uint8_t _cs_pin;

    uint64_t subscribers[64];
    size_t num_subscribers;

    void add_subscriber(uint64_t address);
};