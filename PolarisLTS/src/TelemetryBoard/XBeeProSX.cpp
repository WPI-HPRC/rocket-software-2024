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

    readSDDirectory(); // Read back the directory to show the card is cleared
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

        // We need to have this null character to denote the end of a filename
        sprintf(fileName, "file%02d\0", i);

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
    // Send the following packet to denote the end of the directory
    uint8_t req = 0xCC;
    sendTransmitRequestCommand(0x0013A200423F474C, &req, 1);
}

void XbeeProSX::readFileContents(XBee::ReceivePacket::Struct *frame)
{
    int filenameLength = 0;
    // Add 1 to skip the frame type
    while(frame->data[1 + filenameLength] != '\0')
    {
        filenameLength++;
    }

    // Next, we need to see if there exists a file with the filename specified in the frame (beginning at byte 1 and ending at byte [filenameLength]). If the file exists, read its contents

    int fileLength = 10000;
    char file[fileLength];

    // Three additional bytes are reserved: first is the packet type, next two are for the packet index
    uint8_t maxBytes = XBee::MaxPacketBytes - XBee::TransmitRequest::PacketBytes - 3;
    uint8_t filePacket[maxBytes + 3];
    memset(&filePacket[3], 0, maxBytes);
    filePacket[0] = 0xFC; // "File Contents"
    filePacket[1] = 0; // The first packet
    filePacket[2] = 0;

    uint16_t packetIndex = 0;
    uint16_t packetsRequired = (uint16_t)((float)fileLength / ((float)maxBytes) + 1);

    uint32_t numPacketsReq = 0x000000FC | packetsRequired << 8; // This is a uint32 for convenience of creating this packet, but the actual information we want to send is not a uint32

    // Send a packet telling us how many packets will be required
    sendTransmitRequestCommand(0x0013A200423F474C, (uint8_t *)&numPacketsReq, 4);

    for (int i = 0; i < fileLength; i++)
    {
        for (int n = 0; n < maxBytes && i < fileLength; n++, i++)
        {
            filePacket[i+3] = (uint8_t)file[i];
        }
        sendTransmitRequestCommand(0x0013A200423F474C, filePacket, sizeof(filePacket));
        *(uint16_t *)&filePacket[1] = ++packetIndex;
        memset(&filePacket[3], 0, maxBytes);
    }

    sendTransmitRequestCommand(0x0013A200423F474C, filePacket, sizeof(filePacket));

    // Send the following packet to denote the end of the packet
    uint8_t req = 0xFC;
    sendTransmitRequestCommand(0x0013A200423F474C, &req, 1);
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
        case 0xFC: // "File Contents"
            readFileContents(frame);
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
