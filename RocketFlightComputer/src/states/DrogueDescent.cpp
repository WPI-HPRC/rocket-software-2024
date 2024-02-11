#include "State.h"
#include "DrogueDescent.h"
#include "Sensors.h"

int sumDrogueDescentVel = 0;
const float METER_CONVERSION = 0.3048;

float DROGUE_DESCENT_THRESHOLD; // tbd
DrogueDescent::DrogueDescent(struct Sensors *sensors, StateEstimator *stateEstimator, FlashChip *flashChip) : State(sensors, stateEstimator, flashChip) {}


DrogueDescent::DrogueDescent() 
{

}

void DrogueDescent::initialize_impl() 
{
	this->currentTime = millis();
}

void DrogueDescent::loop_impl() 
{
	while(altitude < (1500 * METER_CONVERSION)){
		logData();

    	// ABORT Case
    	// 10 second timeout
    	if (/*New abort condition*/)
    	{
        	nextState_impl('a');
			break;
    	}
           
    	// detect altitude drop below 1500ft
    	if (altitude < (1500 * METER_CONVERSION))
    	{
        	nextState_impl('m');
        	break;
    	}
	}
            
}

State *DrogueDescent::nextState_impl(char nextStateChar) 
{
	if(nextStateChar == 'a')
	{
		return *Abort; //do we have an abort state for the new state machine?
	}
	else if(nextStateChar == 'm')
	{
		return *MainDescent; 
	}

	return nullptr;
}
