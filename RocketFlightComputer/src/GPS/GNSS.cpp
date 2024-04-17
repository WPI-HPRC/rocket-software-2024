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
#include "GNSS.h"

GNSS::GNSS() {}

bool GNSS::init() {
    if (!this->gnss.begin()) {
        return false;
    }

    this->gnss.setI2COutput(COM_TYPE_UBX);
    this->gnss.setNavigationFrequency(5);
    this->gnss.setAutoPVT(true);

    // Enable the jamming / interference monitor
    // UBX_CFG_ITFM_data_t jammingConfig; // Create storage for the jamming configuration
    // if (gnss.getJammingConfiguration(&jammingConfig)) // Read the jamming configuration
    // {
    //     Serial.print(F("[GNSS] The jamming / interference monitor is "));
    //     if (jammingConfig.config.bits.enable == 0) // Check if the monitor is already enabled
    //     Serial.print(F("not "));
    //     Serial.println(F("enabled"));

    //     if (jammingConfig.config.bits.enable == 0) // Check if the monitor is already enabled
    //     {
    //     Serial.print(F("[GNSS] Enabling the jamming / interference monitor: "));
    //     (jammingConfig.config.bits.enable = 1); // Enable the monitor
    //     if (gnss.setJammingConfiguration(&jammingConfig)) // Set the jamming configuration
    //         Serial.println(F(" -> success"));
    //     else
    //         Serial.println(F(" -> failed!"));
    //     }
    // }

    // this->gnss.saveConfiguration();

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

bool GNSS::dataReady() {
    return (gnss.getPVT() && (gnss.getInvalidLlh() == false));
};

void GNSS::printHwStatus() {
    UBX_MON_HW_data_t hwStatus;

    if(gnss.getHWstatus(&hwStatus)) {
        Serial.println(F("Hardware status (UBX_MON_HW):"));

        Serial.print(F("Jamming state: "));
        Serial.print(hwStatus.flags.bits.jammingState);
        if (hwStatus.flags.bits.jammingState == 0)
        Serial.println(F(" = unknown / disabled"));
        else if (hwStatus.flags.bits.jammingState == 1)
        Serial.println(F(" = ok"));
        else if (hwStatus.flags.bits.jammingState == 2)
        Serial.println(F(" = warning"));
        else // if (hwStatus.flags.bits.jammingState == 3)
        Serial.println(F(" = critical!"));

        Serial.print(F("Noise level: "));
        Serial.println(hwStatus.noisePerMS);
        
        Serial.print(F("AGC monitor: "));
        Serial.println(hwStatus.agcCnt);
        
        Serial.print(F("CW jamming indicator: "));
        Serial.println(hwStatus.jamInd);

        Serial.println();
    };
}

int GNSS::getMagneticDeclination() {
    if(gnss.getHeadVehValid()) {
        return gnss.getMagDec();
    } else {
        return -1;
    }
    
}

uint32_t GNSS::getEpochTime() {
    return gnss.getUnixEpoch();
}
