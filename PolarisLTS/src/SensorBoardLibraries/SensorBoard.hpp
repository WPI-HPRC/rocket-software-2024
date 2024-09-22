#include <SensorBoardLibraries/IMU/IMU_SB.h>
#include <SensorBoardLibraries/Barometer/Barometer_SB.h>
#include <SensorBoardLibraries/Magnetometer/Magnetometer_SB.h>
#include "Config.h"
#include "Sensor_Frames.hpp"
// #include <libs/MMC5983/SparkFun_MMC5983MA_Arduino_Library.h>

/*
    @author Samay Govani
    @brief Sensorboard class contains all the sensor objects and functions to read data from them and store it in a buffer
    @details Worcester Polytechnic Institute High Power Rocketry Club
*/
class Sensorboard{
    private:
    MS5611 barometer = MS5611(Wire, BARO_I2C_ADDRESS); // Barometer
    ICM42688P imu = ICM42688P(Wire, IMU_I2C_ADDRESS); // IMU
    // SFE_MMC5983MA mag;
    MMC5983MA mag = MMC5983MA(Wire, MAG_I2C_ADDRESS); // Magnetometer
    // MMC5983MA mag = MMC5983MA(Wire, MAG_I2C_ADDRESS); // Magnetometer
    // SFE_UBLOX_GNSS gps; // GPS

    uint8_t Buffer[29] = {0};
    uint32_t MagData[3] = {0}; // x,y,z

    public:
    Sensorboard(){};
    SensorFrame Inertial_Baro_frame;

    /*
        @brief Sets up all the sensors
        @details Returns true if all sensors are setup correctly
    */
    bool setup(){
        if (!imu.setup()) return false;
        if (!barometer.setup()) return false;
        if (!mag.setup()) return false;
        // if(!mag.begin()) return false;

        // mag.softReset();

        return true;
    }
    
    /*
        @brief Reads data from all sensors and stores it in the buffer
        @details The buffer is a 29 byte array that contains the data from the sensors in the following order:
        IMU: 12 bytes Accelerometer: 6 bytes (H byte, L Byte for each axis) Gyroscope: 6 bytes (H byte, L Byte for each axis) in X,Y,Z order
        Barometer: 6 bytes Pressure: 3 bytes Temperature: 3 bytes
        Magnetometer: 7 bytes X: 2 bytes Y: 2 bytes Z: 2 bytes Extra Byte for 18 Bit Resolution for each axis
        Time: 4 bytes milliseconds since the program started
    */
    void readInertialSensors(){
        // Store sensor data in buffer
        imu.readSensor(Buffer,0);
        barometer.readSensor(Buffer,12);
        mag.sensorRead(MagData);

        // mag.readSensor(*magData);
        this->ProcessBuffer();
    }

    void ProcessBuffer(){
        // Process the buffer and store the data in the frame, once the frame is updated set newFrame to true

        // Accelerometer
        Inertial_Baro_frame.ac_x = ICM42688P::processAxis(ICM42688P::processHighLowByte(Buffer[0],Buffer[1]), 2048.0);
        Inertial_Baro_frame.ac_y = ICM42688P::processAxis(ICM42688P::processHighLowByte(Buffer[2],Buffer[3]), 2048.0);
        Inertial_Baro_frame.ac_z = ICM42688P::processAxis(ICM42688P::processHighLowByte(Buffer[4],Buffer[5]), 2048.0);

        // Gyroscope
        Inertial_Baro_frame.gy_x = ICM42688P::processAxis(ICM42688P::processHighLowByte(Buffer[6],Buffer[7]), 16.4);
        Inertial_Baro_frame.gy_y = ICM42688P::processAxis(ICM42688P::processHighLowByte(Buffer[8],Buffer[9]), 16.4);
        Inertial_Baro_frame.gy_z = ICM42688P::processAxis(ICM42688P::processHighLowByte(Buffer[10],Buffer[11]), 16.4);

        // Barometer
        barometer.calculatePressureAndTemperature(MS5611::processHighMidLowByte(Buffer[12],Buffer[13],Buffer[14]),MS5611::processHighMidLowByte(Buffer[15],Buffer[16],Buffer[17]),&Inertial_Baro_frame.Pressure,&Inertial_Baro_frame.Temperature);

        // uint32_t magXRaw = mag.getMeasurementX();
        // uint32_t magYRaw = mag.getMeasurementY();
        // uint32_t magZRaw = mag.getMeasurementZ();

        Inertial_Baro_frame.mag_x = MagData[0] - 133160.0;
        Inertial_Baro_frame.mag_x /= 133160.0;

        Inertial_Baro_frame.mag_y = MagData[1] - 131450.0;
        Inertial_Baro_frame.mag_y /= 131450.0;

        Inertial_Baro_frame.mag_z = MagData[2] - 133030.0;
        Inertial_Baro_frame.mag_z /= 133030.0;

        Inertial_Baro_frame.mag_x *= 8; // [Gauss]
        Inertial_Baro_frame.mag_y *= 8; // [Gauss]
        Inertial_Baro_frame.mag_z *= 8; // [Gauss]

        // Inertial_Baro_frame.mag_x = mag.getMeasurementX();
        // Inertial_Baro_frame.mag_y = mag.getMeasurementY();
        // Inertial_Baro_frame.mag_z = mag.getMeasurementZ();

        // Inertial_Baro_frame.mag_x = MagData[0];
        // Inertial_Baro_frame.mag_y = MagData[1];
        // Inertial_Baro_frame.mag_z = MagData[2];

        // mag.calculateCalibratedValues(MagData[0], MagData[1], MagData[2], &Inertial_Baro_frame.mag_x, &Inertial_Baro_frame.mag_y, &Inertial_Baro_frame.mag_z);

    }
};
