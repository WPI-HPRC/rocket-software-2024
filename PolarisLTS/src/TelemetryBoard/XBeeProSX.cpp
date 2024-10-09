#include "XBeeProSX.h"
#include <cstdarg>

XbeeProSX::XbeeProSX(uint8_t cs_pin) : _cs_pin(cs_pin), XBeeDevice(SerialInterface::SPI)
{
    sendTransmitRequestsImmediately = true;
    sendFramesImmediately = true;
}

void XbeeProSX::start()
{
    pinMode(_cs_pin, OUTPUT);
}

void XbeeProSX::writeBytes(const char *data, size_t length_bytes)
{
    digitalWrite(_cs_pin, LOW);
    // #define W(b) SPI.transfer(b)
    // W(0x7e);
    // W(0x00);
    // W(0x0f);
    // W(0x10);
    // W(0x01);
    // W(0x00);
    // W(0x13);
    // W(0xa2);
    // W(0x00);
    // W(0x42);
    // W(0x3f);
    // W(0x47);
    // W(0x4c);
    // W(0xff);
    // W(0xfe);
    // W(0x00);
    // W(0xc0);
    // W(0xaa);
    // W(0xbe);
    for (size_t i = 0; i < length_bytes; i++)
    {
        Serial.printf("%02x ", data[i]);
        uint8_t byte = SPI.transfer(data[i]);
    }
    Serial.printf("\n");
    
    digitalWrite(_cs_pin, HIGH);
}

void XbeeProSX::handleReceivePacket(XBee::ReceivePacket::Struct *frame)
{

}

void XbeeProSX::handleReceivePacket64Bit(XBee::ReceivePacket64Bit::Struct *frame)
{

}

void XbeeProSX::incorrectChecksum(uint8_t calculated, uint8_t received)
{

}

void XbeeProSX::didCycle()
{

}

void XbeeProSX::log(const char *format, ...)
{
    va_list args;
    va_start(args, format);  

    va_end(args);
}
