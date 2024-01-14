#pragma once

#include <Arduino.h>
// #include <Controls/EKF/KalmanFilter.h>
#include <ControlsOld/EKF/StateEstimator.h>
#include <ArduinoEigen.h>

#define _STATE_CLASS_IMPLS_          \
private:                             \
    void initialize_impl() override; \
    void loop_impl() override;       \
    State *nextState_impl() override;
/**
 * @brief Abstract class representing a rocket state.
 */
class State
{
public:
    /**
     * @brief Code to be run once when the state begins.
     */
    void initialize();
    /**
     * @brief Code to be run every iteration of the loop while the rocket is in this state.
     */
    void loop();
    /**
     * @brief Code run every iteration of the loop to determine the next state to transition to.
     * @return The pointer to the next state or nullptr if the state has not changed.
     */
    State *nextState();
    virtual ~State() {}

    String name = "";

    // Shared Variable
    SensorFrame sensorData;

    // Eigen::Vector<float, 10> currentState;
    BLA::Matrix<4> currentState;

    // struct {
    //     float accelX = 0.0;
    //     float accelY = 0.0;
    //     float accelZ = 0.0;
    //     float gyroX = 0.0;
    //     float gyroY = 0.0;
    //     float gyroZ = 0.0;
    //     uint32_t magX = 0.0;
    //     uint32_t magY = 0.0;
    //     uint32_t magZ = 0.0;
    //     float altitude = 0.0;
    //     float pressure = 0.0;
    //     float q = 0.0;
    //     float i = 0.0;
    //     float j = 0.0;
    //     float k = 0.0;
    //     uint32_t timestamp;
    // } telemPacket;

    struct TelemPacket {
        float accelX;
        float accelY;
        float accelZ;
        float gyroX;
        float gyroY;
        float gyroZ;
        uint32_t magX;
        uint32_t magY;
        uint32_t magZ;
        float altitude;
        float pressure;
        float q;
        float i;
        float j;
        float k;
        long timestamp;
    };

    TelemPacket telemPacket;

    static float pressureToAltitude(float pressure);

protected:
    //! @brief number of milliseconds since the initialize call
    long long currentTime = 0;
    //! @brief number of milliseconds since the last loop call
    long long deltaTime = 0;

private:
    //! @brief number of milliseconds from boot to the initialize call
    long long startTime = 0;
    //! @brief number of milliseconds from boot to the previous loop call
    long long lastLoopTime = 0;
    virtual void initialize_impl() = 0;
    virtual void loop_impl() = 0;
    virtual State *nextState_impl() = 0;
};