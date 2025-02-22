#include "XBeeProSX.h"
#include <cstdarg>

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
        uint8_t byte = SPI.transfer(data_io[i]);
        data_io[i] = (char)byte;
        // Serial.printf("%x ", byte);
    }
    // Serial.println();
    
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

bool XbeeProSX::canReadSPI()
{
    return digitalRead(33);;
}

void XbeeProSX::log(const char *format, ...)
{
    va_list args;
    va_start(args, format);  

    va_end(args);
}
