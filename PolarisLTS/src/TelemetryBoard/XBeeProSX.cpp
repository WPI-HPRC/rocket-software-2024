#include "XBeeProSX.h"
#include <cstdarg>
#include "utility.hpp"

XbeeProSX::XbeeProSX(uint8_t cs_pin) : _cs_pin(cs_pin), XBeeDevice(SerialInterface::SPI)
{
    sendTransmitRequestsImmediately = true;
    sendFramesImmediately = true;
    serialInterface = SerialInterface::SPI;
}

void XbeeProSX::start()
{
    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);

    pinMode(33, INPUT);
}

void XbeeProSX::writeBytes_spi(char *data_io, size_t length_bytes)
{
    digitalWrite(_cs_pin, LOW);
    for (size_t i = 0; i < length_bytes; i++)
    {
        data_io[i] = (char)SPI.transfer(data_io[i]);
    }
    digitalWrite(_cs_pin, HIGH);
}

void XbeeProSX::handleReceivePacket(XBee::ReceivePacket::Struct *frame)
{
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
    digitalWrite(_cs_pin, LOW);
    for (size_t i = 0; i < length_bytes; i++)
    {
        buffer[i] = SPI.transfer(0x00);
    }
    digitalWrite(_cs_pin, HIGH);
}

bool XbeeProSX::canReadSPI()
{
    return digitalRead(33) == LOW;
}

void XbeeProSX::log(const char *format, ...)
{
    va_list args;
    va_start(args, format);  

    va_end(args);
}
