#include "XBeeProSX.h"
#include <cstdarg>
#include "utility.hpp"

XbeeProSX::XbeeProSX(uint8_t cs_pin, uint8_t attn_pin) : cs_pin(cs_pin), attn_pin(attn_pin), XBeeDevice(SerialInterface::SPI)
{
    // The following two flags may be able to be changed once we get the frame queue code working
    sendTransmitRequestsImmediately = true;
    sendFramesImmediately = true;
    serialInterface = SerialInterface::SPI;
}

void XbeeProSX::start()
{
    pinMode(cs_pin, OUTPUT);
    // Write the ChipSelect pin HIGH (no data to be sent). When data is ready to be sent/received, we pull this pin LOW
    digitalWrite(cs_pin, HIGH);

    pinMode(attn_pin, INPUT);
}

void XbeeProSX::writeBytes_spi(char *data_io, size_t length_bytes)
{
    digitalWrite(cs_pin, LOW);
    for (size_t i = 0; i < length_bytes; i++)
    {
        data_io[i] = (char)SPI.transfer(data_io[i]);
    }
    digitalWrite(cs_pin, HIGH);
}

void XbeeProSX::handleReceivePacket(XBee::ReceivePacket::Struct *frame)
{
    // TODO: Add more types of packets and figure out a way to prevent control commands from being executed during flight
    if(frame->data[0] == 0xAB)
    {
        airbrakesServo.write(*(uint16_t*)&frame->data[1]);
    }
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

void XbeeProSX::readBytes_spi(uint8_t *buffer, size_t length_bytes)
{
    digitalWrite(cs_pin, LOW);
    for (size_t i = 0; i < length_bytes; i++)
    {
        buffer[i] = SPI.transfer(0x00);
    }
    digitalWrite(cs_pin, HIGH);
}

bool XbeeProSX::canReadSPI()
{
    return digitalRead(attn_pin) == LOW;
}

void XbeeProSX::log(const char *format, ...)
{
    va_list args;
    va_start(args, format);  

    va_end(args);
}
