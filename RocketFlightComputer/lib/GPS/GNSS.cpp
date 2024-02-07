/**
 * @file GNSS.cpp
 * @author Ritvik Garg
 * @brief GNSS base code
 * @version 1.0
 * @date 2023-10-07
 *
 * @copyright Copyright (c) 2023
 */
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <GNSS.h>

GNSS::GNSS() {}

bool GNSS::init() {
    if (!this->gnss.begin()) {
        return false;
    }

    this->gnss.setI2COutput(COM_TYPE_UBX);
    this->gnss.setNavigationFrequency(40);
    this->gnss.setAutoPVT(true);
    this->gnss.saveConfiguration();

    return true;
}

void GNSS::outputData() {
    double lat = getLatitude();
    double lon = getLongitude();
    Serial.println("Latitude: " + String(lat) + ". Longitude: " + String(lon));
}

float GNSS::getLatitude() {
    return gnss.getLatitude();
}

float GNSS::getLongitude() {   
    return gnss.getLongitude();
}

uint8_t GNSS::getSatellites() {
    return gnss.getSIV();
}

bool GNSS::getLockStatus() {
    return gnss.getGnssFixOk();
}

float GNSS::getAltMSL() {
    return gnss.getAltitudeMSL();
};

float GNSS::getAltAGL() {
    return gnss.getAltitude();
};

String GNSS::getTime() {
    String timeString;
    
    String hour = String(gnss.getHour());
    String minute = String(gnss.getMinute());
    String second = String(gnss.getSecond());

    timeString += hour + ":" + minute + ":" + second;
}