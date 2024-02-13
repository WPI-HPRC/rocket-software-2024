#pragma once
#include "Sensors.h"

#define _STATE_CLASS_IMPLS_          \
private:                             \
    void initialize_impl() override; \
    void loop_impl() override;       \
    State *nextState_impl() override;

#include "Arduino.h"
#include "../utility.hpp"
#include <ArduinoEigen.h>
#include <BasicLinearAlgebra.h>
#include <Flash.h>
#include <EKF.h>
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

    // FIXME: I think these should be protected
    Utility::SensorPacket sensorPacket;

    // Eigen::Vector<float, 10> x_state;
    BLA::Matrix<10> x_state;

protected:
    //! @note Constructor to be called from subclasses to initialize the `sensors` object
    State(struct Sensors *sensors, StateEstimator *stateEstimator);
    //! @brief number of milliseconds since the initialize call
    long long currentTime = 0;
    //! @brief number of milliseconds since the last loop call
    long long deltaTime = 0;
    //! @brief loop count since initialization
    long long loopCount = 0;
    StateEstimator *stateEstimator;
    //! @brief "global" sensors object
    struct Sensors *sensors;

private:
    //! @brief number of milliseconds from boot to the initialize call
    long long startTime = 0;
    //! @brief number of milliseconds from boot to the previous loop call
    long long lastLoopTime = 0;
    virtual void initialize_impl() = 0;
    virtual void loop_impl() = 0;
    virtual State *nextState_impl() = 0;
};
