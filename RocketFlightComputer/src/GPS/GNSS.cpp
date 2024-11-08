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

    // this->gnss.setI2COutput(COM_TYPE_UBX);
    this->gnss.setI2COutput(COM_TYPE_UBX);
    this->gnss.setNavigationFrequency(40);
    this->gnss.setAutoPVT(true);

    //Disable Beidou
    // this->gnss.addCfgValset(UBLOX_CFG_SIGNAL_BDS_ENA, 0);

    // //Disable Galileo
    // this->gnss.addCfgValset(UBLOX_CFG_SIGNAL_GAL_E1_ENA, 0);

    // //Enable GPS
    // this->gnss.addCfgValset(UBLOX_CFG_SIGNAL_GPS_ENA, 1);
    // this->gnss.addCfgValset(UBLOX_CFG_SIGNAL_GPS_L1CA_ENA, 1);

    // //Enable SBAS
    // this->gnss.addCfgValset(UBLOX_CFG_SIGNAL_SBAS_ENA, 1);
    // this->gnss.addCfgValset(UBLOX_CFG_SIGNAL_SBAS_L1CA_ENA, 1);

    // //Enable QZSS
    // this->gnss.addCfgValset(UBLOX_CFG_SIGNAL_QZSS_ENA, 1);
    // this->gnss.addCfgValset(UBLOX_CFG_SIGNAL_QZSS_L1CA_ENA, 1);
    // this->gnss.addCfgValset(UBLOX_CFG_SIGNAL_QZSS_L1S_ENA, 1);

    // //Enable GLONASS
    // this->gnss.addCfgValset(UBLOX_CFG_SIGNAL_GLO_ENA, 1);
    // this->gnss.addCfgValset(UBLOX_CFG_SIGNAL_GLO_L1_ENA, 1);

    // this->gnss.addCfgValset(UBLOX_CFG_HW_ANT_CFG_VOLTCTRL, 1);

    // Serial.print("[GNSS] Voltage Control: "); Serial.println(this->gnss.getVal8(UBLOX_CFG_HW_ANT_CFG_VOLTCTRL));

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

int32_t GNSS::getNorthVelocity() {
    return gnss.getNedNorthVel();
}

int32_t GNSS::getEastVelocity() {
    return gnss.getNedEastVel();
}

int32_t GNSS::getDownVelocity() {
    return gnss.getNedDownVel();
}

sfe_ublox_antenna_status_e GNSS::getAntStatus() {
    return gnss.getAntennaStatus();
}