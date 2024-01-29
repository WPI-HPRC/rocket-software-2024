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
#include <SparkFun_u-blox_GNSS_v3.h>

class GNSS
{
public:
    GNSS();
    /**
     * @brief initializes the internal gnss
     */
    void init();
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
    double getLatitude();
    /**
     * @brief Get the longitude value
     *
     * @return the longitude value
     */
    double getLongitude();

private:
    SFE_UBLOX_GNSS gnss;
};
