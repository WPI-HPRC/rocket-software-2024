#include "XBeeProSX.h"

XbeeProSX::XbeeProSX(uint8_t cs_pin) : _cs_pin(cs_pin), XBeeDevice()
{
    sendTransmitRequestsImmediately = true;
    sendFramesImmediately = true;
}

void XbeeProSX::start()
{
    pinMode(_cs_pin, OUTPUT);
}

void XbeeProSX::readBytes(uint8_t *buffer, size_t length_bytes)
{
    *buffer = 0;
}

void XbeeProSX::writeBytes(const char *data, size_t length_bytes)
{
    digitalWrite(_cs_pin, LOW);
    for (size_t i = 0; i < length_bytes; i++)
    {
        uint8_t byte = SPI.transfer(data[i]);
    }
    
    digitalWrite(_cs_pin, HIGH);
}

void XbeeProSX::packetRead()
{

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
    Serial.printf(format, args);

    va_end(args);
}
