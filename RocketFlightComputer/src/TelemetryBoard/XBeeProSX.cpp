#include "XBeeProSX.h"

XbeeProSX::XbeeProSX(uint8_t cs_pin) : _cs_pin(cs_pin) {
    // subscribers = new uint64_t[MAX_SUBSCRIBER_COUNT];
    num_subscribers = 0;
}

void XbeeProSX::add_subscriber(uint64_t address) {
    subscribers[num_subscribers] = address;
    num_subscribers++;
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

    uint8_t *packet = (uint8_t*)calloc(contentLength, 1);

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


    // Serial.println("Packet:");
    // for (size_t i = 0; i < index; i++) {
    //     if (packet[i] < 16) Serial.print("0");
    //     Serial.print(packet[i], HEX);
    //     Serial.print(" ");
    // }
    // Serial.println();
    


    free(packet);

}

ReceivePacket *XbeeProSX::receive() {
    Serial.println("Here 3");
    uint64_t tempChecksum = 0;
    uint8_t currentByte = 0;
    ReceivePacket *packet = (ReceivePacket *)calloc(1, sizeof(ReceivePacket));
    digitalWrite(_cs_pin, LOW);  // Asserts module to receive

    Serial.println("Here 4");

    // Read length high and low byte
    uint16_t length = SPI.transfer(0x00) << 8;
    length |= SPI.transfer(0x00);

    uint8_t frameType = SPI.transfer(0x00);

    packet->length = length - 12;
    packet->data = new uint8_t[length - 12];

    Serial.println("Here 2");

    // Read and store source address (64-bit)
    for (int i = 0; i < 8; i++) {
        currentByte = SPI.transfer(0x00);
        tempChecksum += currentByte;
        packet->address |= (currentByte << (i * 8));
    }

    // Skip reserved bytes
    tempChecksum += SPI.transfer(0x00); 
    tempChecksum += SPI.transfer(0x00);

    // Recieve Options
    tempChecksum += SPI.transfer(0x00);

    // Read the message data
    for (int i = 0; i < packet->length; ++i) {  // 12 bytes already read (Frame type, ID, address, reserved)
        currentByte = SPI.transfer(0x00);
        tempChecksum += currentByte;
        packet->data[i] = currentByte;
    }

    Serial.println("Here 1");

    uint8_t checksum = 0xFF - (tempChecksum & 0xFF);
    uint8_t receivedChecksum = SPI.transfer(0x00);
    digitalWrite(_cs_pin, HIGH);  // De-asserts module

    if(checksum != receivedChecksum) {
        Serial.print("Wrong checksum! Received "); Serial.print(receivedChecksum); Serial.print(", calculated "); Serial.print(checksum);
    }
    else {
        Serial.print("Received message, right checksum!");
    }

        // Read frame type, skip if not 0x90
    if (frameType != 0x90) {
          Serial.println("Wrong frame type received");
        // return packet;
    }

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

    Serial.println("Data available!");

    ReceivePacket *message = receive();

    if(message->length != 9) {// Length of the word "subscribe"
        return;
    } else {
        Serial.print("Unknown Message: "); Serial.println(*message->data);
    }

    return;

    for (uint i = 0; i < num_subscribers; i++) {
        if(subscribers[i] == message->address)
            return;
    }

    add_subscriber(message->address);

    delete[] message->data;
    free(message);
}


void XbeeProSX::sendToSubscribers(const void *data, size_t size) {
    for (uint i = 0; i < num_subscribers; i++) {
        send(subscribers[i], data, size);
    }
}
