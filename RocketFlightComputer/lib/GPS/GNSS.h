/**
 * @file GNSS.h
 * @author Ritvik Garg
 * @brief GNSS base code
 * @version 1.0
 * @date 2023-10-07
 *
 * @copyright Copyright (c) 2023
 */
#pragma once
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

class GNSS
{
public:
    GNSS();
    /**
     * @brief initializes the internal gnss
     */
    bool init();
    /**
     * @brief prints the following to the Serial Monitor: the latitude, longitude, and if either value has changed since the last reading
     *
     * @param delayTime the amount of delay between readings in milliseconds (OPTIONAL, DEFAULT 250)
     */
    void outputData();
    /**
     * @brief Get the latitude value
     *
     * @return the latitude value
     */
    float getLatitude();
    /**
     * @brief Get the longitude value
     *
     * @return the longitude value
     */
    float getLongitude();

    /**
     * @brief Get the status of the GPS Lock
     * 
     * @return true 
     * @return false 
     */
    bool getLockStatus();

    /**
     * @brief Get the number of satellites in view
     * 
     * @return uint8_t 
     */
    uint8_t getSatellites();

    /**
     * @brief Get Altitude AGL
     * 
     * @return float 
     */
    float getAltAGL();

    /**
     * @brief Get the Altitude MSL
     * 
     * @return float 
     */
    float getAltMSL();

    /**
     * @brief Get the current time as a string
     * 
     * @return char* Time
     */
    String getTime();

    /**
     * @brief Checks PVT and Fix Status to see if data is ready
     * 
     * @return true 
     * @return false 
     */
    bool dataReady();

    void printHwStatus();

private:
    SFE_UBLOX_GNSS gnss;

    
};
