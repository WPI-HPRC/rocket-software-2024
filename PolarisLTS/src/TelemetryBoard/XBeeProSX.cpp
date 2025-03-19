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

void XbeeProSX::actuateAirbrakes(XBee::ReceivePacket::Struct *frame)
{
    airbrakesServo.write(*(uint16_t*)&frame->data[1]);
}

void XbeeProSX::clearSD()
{
    if(!sdCardInitialized) return;

    // Add code to clear the SD card
}

void XbeeProSX::readSDDirectory()
{
    if(!sdCardInitialized) return;
    
    uint8_t maxBytes = XBee::MaxPacketBytes - XBee::TransmitRequest::PacketBytes - 2;
    uint8_t fileListPacket[maxBytes + 2];
    fileListPacket[0] = 0xCC; // "Card Contents"
    fileListPacket[1] = 0; // The first packet

    uint8_t packetIndex = 0;
    uint8_t numBytesInPacket = 0;

    // These are all placeholders for looping through an actual directory.
    int numFiles = 99;
    for (int i = 0; i < numFiles; i++)
    {
        int fileNameLength = 7;
        char fileName[fileNameLength];
        sprintf(fileName, "file%02d", i);

        // Check to make sure there is space in the current packet for the new filename
        if ((int)numBytesInPacket + fileNameLength > maxBytes)
        {
            // Send the current frame
           sendTransmitRequestCommand(0x0013A200423F474C, fileListPacket, sizeof(fileListPacket));
            // Clear the list
            memset(&fileListPacket[2], 0, sizeof(fileListPacket) - 2);
            fileListPacket[1] = ++packetIndex;
            numBytesInPacket = 0;
        }
        memcpy(&fileListPacket[numBytesInPacket + 2], (uint8_t *)fileName, fileNameLength);
        numBytesInPacket += fileNameLength;
    }
    sendTransmitRequestCommand(0x0013A200423F474C, fileListPacket, sizeof(fileListPacket));
}

void XbeeProSX::handleReceivePacket(XBee::ReceivePacket::Struct *frame)
{
    // TODO: Add more types of packets and figure out a way to prevent control commands from being executed during flight
    uint8_t packetType = frame->data[0];
    switch(packetType)
    {
        case 0xAB: // "Air Brakes"
            actuateAirbrakes(frame);
        case 0xCC: // "Clear Card"
            clearSD();
        case 0xCD: // "Card Directory"
            readSDDirectory();
        default:
            return;
    };
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
