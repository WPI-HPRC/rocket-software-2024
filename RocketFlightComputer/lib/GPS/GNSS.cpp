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
#include <Wire.h>
#include <GNSS.h>

GNSS::GNSS() {}

/**
 * @brief initializes the GNSS to print to Serial Monitor
 */
void GNSS::initialize()
{
    Wire.begin();
    if (gnss.begin() == false)
    {
        Serial.println(F("u-blox GNSS module not detected at default I2C address. Please check wiring. Freezing."));
        while (1)
            ;
    }
    Serial.println(F("GNSS Initiallized"));
}
/**
 * @brief returns true if the latitude or longitude have changed since the last reading
 *
 * @param lat the current latitude
 * @param lon the current longitude
 * @return true if the latitude or longitude have changed since last reading
 */
boolean GNSS::hasChanged(double lat, double lon)
{
    return lat == prevLat || lon == prevLon;
}
/**
 * @brief prints the following to the Serial Monitor: the latitude, longitude, and if either value has changed since the last reading
 *
 * @param delayTime the amount of delay between readings in milliseconds (OPTIONAL, DEFAULT 250)
 */
void GNSS::outputData(int delayTime)
{
    double lat = getLatitude();
    double lon = getLongitude();
    Serial.println("Latitude: " + String(lat) + ". Longitude: " + String(lon) + ". Changed: " + hasChanged(lat, lon));
    prevLat = lat;
    prevLon = lon;
    delay(delayTime); // Don't pound too hard on the I2C bus
}
/**
 * @brief Get the latitude value
 *
 * @return the latitude value
 */
double GNSS::getLatitude()
{
    return gnss.getLatitude();
}
/**
 * @brief Get the longitude value
 *
 * @return the longitude value
 */
double GNSS::getLongitude()
{
    return gnss.getLongitude();
}
