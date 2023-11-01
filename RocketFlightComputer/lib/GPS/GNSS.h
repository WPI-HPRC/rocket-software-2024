/**
 * @file GNSS.h
 * @author Ritvik Garg
 * @brief GNSS base code
 * @version 1.0
 * @date 2023-10-07
 *
 * @copyright Copyright (c) 2023
 */
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
class GNSS
{
public:
    /**
     * @brief returns the GNSS object
     *
     * @return SFE_UBLOX_GNSS object
     */
    SFE_UBLOX_GNSS getGNSS();
    /**
     * @brief initializes the GNSS to print to Serial Monitor
     */
    void initialize();
    /**
     * @brief returns true if the latitude or longitude have changed since the last reading
     *
     * @param lat the current latitude
     * @param lon the current longitude
     * @return true if the latitude or longitude have changed since last reading
     */
    boolean hasChanged(double lat, double lon);
    /**
     * @brief prints the following to the Serial Monitor: the latitude, longitude, and if either value has changed since the last reading
     *
     * @param delayTime the amount of delay between readings in milliseconds (OPTIONAL, DEFAULT 250)
     */
    void outputData(int delayTime = 250);
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
    SFE_UBLOX_GNSS GNSS;
    double prevLat;
    double prevLon;
};