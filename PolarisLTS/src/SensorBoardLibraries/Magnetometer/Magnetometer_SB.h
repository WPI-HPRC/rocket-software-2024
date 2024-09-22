#include <Arduino.h>
#include <Wire.h>
#include <SensorBoardLibraries/Sensor.h>
#include "MMC5983_Registers.h"

/*
    @author Samay Govani
    @brief MMC5983MA Magnetometer Class for the Sensor Board
    Worcester Polytechnic Institute High Power Rocketry Club
*/

/*
Calibration Data 
*/
const float _A11 = .1249;
const float _A12 = -.0059;
const float _A13 = -.0002;
const float _A21 = -.0059;
const float _A22 = .1252;
const float _A23 = -.0017;
const float _A31 = -.0002;
const float _A32 = .0017;
const float _A33 = .1343;

const uint32_t _B1 = 128670;
const uint32_t _B2 = 130090;
const uint32_t _B3 = 131600;


class MMC5983MA {
    public:
    MMC5983MA(TwoWire &bus,uint8_t address);
    bool setup();
    void sensorRead(uint32_t *MagArray);
    void calculateCalibratedValues(uint32_t Raw_X, uint32_t Raw_Y, uint32_t Raw_Z, float *Calibrated_X, float *Calibrated_Y, float *Calibrated_Z);
    private:
    bool CheckProductID();
    void writeRegister(uint8_t SubAddress, uint8_t data);
    void readRegister(uint8_t SubAddress, int length, uint8_t *data);
    void sensorEnable();
    int I2C_Address;
    TwoWire *I2C_BUS;
};
