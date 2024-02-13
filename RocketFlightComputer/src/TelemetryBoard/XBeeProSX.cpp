#include "XbeeProSX.h"

XbeeProSX::XbeeProSX(uint8_t cs_pin) : _cs_pin(cs_pin) {}

void XbeeProSX::begin() {

    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);

    Serial.begin(9600);

    SPI.begin();
    SPI.beginTransaction(SPISettings(6000000, MSBFIRST, SPI_MODE0));

}

bool XbeeProSX::isDataAvailable() {

    digitalWrite(_cs_pin, LOW);  // Asserts module to check data
    
    bool available = (SPI.transfer(0x00) == 0x7E);  // Check if start delimiter is present
 
    digitalWrite(_cs_pin, HIGH);  // De-asserts module

    return available;

}

void XbeeProSX::send(const std::vector<uint8_t> &address, const std::string &message) {

    size_t messageLength = message.length();
    size_t contentLength = messageLength + 14; // Length calculation

    uint8_t *packet = new uint8_t[contentLength + 4]; // +4 for start delimiter, length, and checksum

    size_t index = 0;

    packet[index++] = 0x7E; // Start delimiter

    packet[index++] = (contentLength >> 8) & 0xFF; // Length high byte
    packet[index++] = contentLength & 0xFF;       // Length low byte

    packet[index++] = 0x10; // Frame type
    packet[index++] = 0x00; // Frame ID

    // Copy address
    for (size_t i = 0; i < address.size(); ++i) {
        packet[index++] = address[i];
    }

    packet[index++] = 0xFF; // Reserved
    packet[index++] = 0xFE; // Reserved

    packet[index++] = 0x00; // Broadcast radius

    packet[index++] = 0x03; // Options byte

    // Copy message
    for (size_t i = 0; i < messageLength; ++i) {
        packet[index++] = static_cast<uint8_t>(message[i]);
    }

    // Calculate and append checksum
    uint8_t checksum = 0xFF;

    for (size_t i = 3; i < contentLength + 3; ++i) { // Start from byte after length field
        checksum -= packet[i];
    }

    packet[index++] = checksum;

    digitalWrite(_cs_pin, LOW);  // Asserts module to transmit

    // Send the packet over SPI
    for (size_t i = 0; i < contentLength + 4; ++i) {
        SPI.transfer(packet[i]);
    }

    digitalWrite(_cs_pin, HIGH);  // De-asserts module

    Serial.println("Packet:");
    for (size_t i = 0; i < contentLength + 4; ++i) {
        if (packet[i] < 16) Serial.print("0");
        Serial.print(packet[i], HEX);
        Serial.print(" ");
    }
    Serial.println();


    delete[] packet; // Free allocated memory

}

std::pair<std::vector<uint8_t>, std::string> XbeeProSX::receive() {

    std::vector<uint8_t> address;
    std::string message;

    digitalWrite(_cs_pin, LOW);  // Asserts module to receive

    // Read length high and low byte
    uint16_t length = SPI.transfer(0x00) << 8;
    length |= SPI.transfer(0x00);

    // Read frame type, skip if not 0x90
    if (SPI.transfer(0x00) != 0x90) {

        digitalWrite(_cs_pin, HIGH);  // De-asserts module

        return {address, message};

    }


    // Read and store source address (64-bit)
    for (int i = 0; i < 8; ++i) {
        address.push_back(SPI.transfer(0x00));
    }

    // Skip reserved bytes
    SPI.transfer(0x00); 
    SPI.transfer(0x00);

    // Recieve Options
    SPI.transfer(0x00);

    // Read the message data
    for (int i = 0; i < length - 12; ++i) {  // 12 bytes already read (Frame type, ID, address, reserved)
        message += static_cast<char>(SPI.transfer(0x00));
    }

    // Skip checksum
    SPI.transfer(0x00);
    digitalWrite(_cs_pin, HIGH);  // De-asserts module

    return {address, message};

}

void XbeeProSX::broadcast(const std::string &message) {

    std::vector<uint8_t> broadcastAddress = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF
    };

    send(broadcastAddress, message);
    
}

void XbeeProSX::updateSubscribers() {

    if (isDataAvailable()) {

        auto data = receive();

        std::vector<uint8_t> address = data.first;
        std::string message = data.second;

        if (!address.empty() && message == "subscribe") {
            Serial.print("Added: "); Serial.println(message.c_str());
            // Check if the address is already in the list of subscribers
            if (std::find(subscribers.begin(), subscribers.end(), address) == subscribers.end()) {
                subscribers.push_back(address); // Add new subscriber
            }

        }

    }

}


void XbeeProSX::sendToSubscribers(const std::string &message) {

    for (const auto& subscriberAddress : subscribers) {
        send(subscriberAddress, message);
    }

}