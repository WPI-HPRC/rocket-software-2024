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

void GNSS::init()
{
    gnss.begin();
    Serial.println(F("GNSS Initiallized"));
}

void GNSS::outputData()
{
    double lat = getLatitude();
    double lon = getLongitude();
    Serial.println("Latitude: " + String(lat) + ". Longitude: " + String(lon));
}

double GNSS::getLatitude()
{
    return gnss.getLatitude();
}

double GNSS::getLongitude()
{
    return gnss.getLongitude();
}
