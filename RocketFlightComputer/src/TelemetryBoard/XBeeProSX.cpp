#include "XbeeProSX.h"

XbeeProSX::XbeeProSX(uint8_t cs_pin) : _cs_pin(cs_pin) {
    subscribers = (uint64_t *)malloc(sizeof(uint64_t));
    num_subscribers = 0;
}

void XbeeProSX::add_subscriber(uint64_t address) {
    uint64_t *newList = (uint64_t *)calloc(num_subscribers + 1, sizeof(uint64_t));

    memcpy(newList, subscribers, num_subscribers * sizeof(uint64_t));

    num_subscribers += 1;
    newList[num_subscribers] = address;

    free(subscribers);
    subscribers = newList;
}

void XbeeProSX::begin() {

    pinMode(_cs_pin, OUTPUT);
}

bool XbeeProSX::isDataAvailable() {

    digitalWrite(_cs_pin, LOW);  // Asserts module to check data
    
    bool available = (SPI.transfer(0x00) == 0x7E);  // Check if start delimiter is present
 
    digitalWrite(_cs_pin, HIGH);  // De-asserts module

    return available;

}

void XbeeProSX::send(uint64_t address, const void * data, size_t size_bytes) {

    size_t contentLength = size_bytes + 14; // +4 for start delimiter, length, and checksum, +8 for address

    uint8_t *packet = (uint8_t*)calloc(0, contentLength);

    size_t index = 0;

    packet[index++] = 0x7E; // Start delimiter

    packet[index++] = (contentLength >> 8) & 0xFF; // Length high byte
    packet[index++] = contentLength & 0xFF;       // Length low byte

    packet[index++] = 0x10; // Frame type
    packet[index++] = 0x01; // Frame ID

    for (int i = 0; i < 8; i++) {
        packet[index++] = (address >> ((7 - i) * 8)) & 0xFF; 
    }

    packet[index++] = 0xFF; // Reserved
    packet[index++] = 0xFE; // Reserved

    packet[index++] = 0x00; // Broadcast radius

    packet[index++] = 0x00; // Options byte

    memcpy(&packet[index++], data, size_bytes);

    index += size_bytes - 1; // Subtract 1 from packet index to start checksum byte at the last index of packet
    
    // Initalize the checksum
    size_t checksum_temp = 0;

    for (size_t i = 3; i < index; i++) { // Start from byte after length field
        // Serial.print("Byte ["); Serial.print(i); Serial.print("]: "); Serial.println(packet[i], HEX);
        checksum_temp += packet[i];
    }

    uint8_t checksum = checksum_temp & 0xFF;

    checksum = 0xFF - checksum;

    // Serial.println(checksum, HEX);

    packet[index++] = checksum;

    digitalWrite(_cs_pin, LOW);  // Asserts module to transmit

    // Send the packet over SPI
    for (size_t i = 0; i < index; i++) {
        SPI.transfer(packet[i]);
    }

    digitalWrite(_cs_pin, HIGH);  // De-asserts module


    Serial.println("Packet:");
    for (size_t i = 0; i < index; i++) {
        if (packet[i] < 16) Serial.print("0");
        Serial.print(packet[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    


    delete[] packet; // Free allocated memory

}

ReceivePacket *XbeeProSX::receive() {

    ReceivePacket *packet = (ReceivePacket *)calloc(0, sizeof(ReceivePacket));
    digitalWrite(_cs_pin, LOW);  // Asserts module to receive

    // Read length high and low byte
    uint16_t length = SPI.transfer(0x00) << 8;
    length |= SPI.transfer(0x00);

    // Read frame type, skip if not 0x90
    if (SPI.transfer(0x00) != 0x90) {
        digitalWrite(_cs_pin, HIGH);  // De-asserts module

        return packet;
    }

    packet->length = length - 12;
    packet->data = new uint8_t[length - 12];

    // Read and store source address (64-bit)
    for (int i = 0; i < 8; i++) {
        packet->address |= (SPI.transfer(0x00) << (i * 8));
    }

    // Skip reserved bytes
    SPI.transfer(0x00); 
    SPI.transfer(0x00);

    // Recieve Options
    SPI.transfer(0x00);

    // Read the message data
    for (int i = 0; i < packet->length; ++i) {  // 12 bytes already read (Frame type, ID, address, reserved)
        packet->data[i] = SPI.transfer(0x00);
    }

    // Skip checksum
    SPI.transfer(0x00);
    digitalWrite(_cs_pin, HIGH);  // De-asserts module

    // Serial.println(*packet->data);

    for (int i = 0; i < packet->length; i++)
    {
        Serial.println(packet->data[i]);
    }

    return packet;

}

void XbeeProSX::broadcast(const void *data, size_t size) {
    send(0x000000000000FFFF, data, size);
    
}

void XbeeProSX::updateSubscribers() {

    if (!isDataAvailable())
        return;

    ReceivePacket *message = receive();

    if(message->length != 9) {// Length of the word "subscribe"
        return;
    } else {
        Serial.print("Unknown Message: "); Serial.println(*message->data);
    }

    for (uint i = 0; i < num_subscribers; i++) {
        if(subscribers[i] == message->address)
            return;
    }

    add_subscriber(message->address);
}


void XbeeProSX::sendToSubscribers(const void *data, size_t size) {

    for (uint i = 0; i < num_subscribers; i++) {
        send(subscribers[i], data, size);
    }
}