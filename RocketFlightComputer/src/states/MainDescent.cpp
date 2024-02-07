#include "MainDescent.h"
#include "State.h"
#include "Sensors.h"

MainDescent::MainDescent(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip) : State(sensors, stateEstimator, flashChip) {}

void MainDescent::initialize_impl() {}

void MainDescent::loop_impl() {    
    // Data is already logged within the state class    

    // Altitude below 20 meters
    // Altitude value gets updated in sensor reading function

    //Create buffer of last 10 altitude values
    LPS25_data curr_pressure = sensors->barometer->read();
    curr_altitude = pressureToAltitude(curr_pressure.pressure);
    // Move all previous buffer values up one and add new value to the front
    for(int i = 8; i>0; i--)
    {
        altitudeBuffer[i+1] = altitufeBuffer[i];
    }
    altitudeBuffer[0] = curr_altitude;
    
    
    if(altitudeBuffer[9] != 0){
        // Take running average value
        float avg_sum = 0.0;
        for (int i = 0; i < 10; i++)
        {
            avg_sum += altitudeBuffer[i];
        }
        float avg = avg_sum / 10.0;

        // Calculate mean difference from average
        float avg_std_sum = 0.0;
        for (int i = 0; i < 10; i++)
        {
            avg_std_sum += abs(altitudeBuffer[i] - avg);
        }
        float avg_std = avg_std_sum / 10.0;

        // If altitude is nearly constant and below landing threshold
        if (avg_std < 5 && curr_altitude <= LAND_THRESHOLD)
        {            
            nextState();
        }
    }
    // timeout after 100 seconds if other checks fail
    if (timeout(MAIN_DESCENT_TIMEOUT))
    {
        nextState();
    }
}

State *MainDescent::nextState_impl() {
    float state_start = millis();
	return nullptr;
}
