#include "controller.h"
#include "gradDescent.h"




// Define the forceToAirBrake function
double Controller::forceToAirBrake(double force) {
    
    // get the equation from the simulations team 
    double acutation = 0.0;

    // Placeholder: Just returning the force for demonstration purposes
    return acutation;
}

double Controller::ObjectFcn(std::vector<double> *funcLoc, SystemState currState)
{

    // predictedAlt = currState.z + (currState.vz * lookahead * dt) - ((0.5 * controlInput * std::sin(currState.theta) * currState.vx * std::sin(currState.phi) * currState.vy * std::pow(lookahead * dt, 2))) / mass;
    // optimalTrajectory = -100 * std::pow(lookaheadTime, 2) + 2000 * lookaheadTime; (FILL IN LATER)
    // altError = predictedAlt - optimalTrajectory(currState);
    // objFcn: 0.9 * (std::pow(altError, 2)) + 0.1 * (std::pow(controlInput - prevU, 2))

	double controlInput = funcLoc->at(0);
	return 0.8 * (std::pow(currState.z + (currState.vz * lookahead * dt) - ((0.5 * controlInput * std::sin(currState.theta) * currState.vx * std::sin(currState.phi) * currState.vy * std::pow(lookahead * dt, 2))) / mass - 100 * std::pow((lookahead * dt) + currState.eTime, 2) + 2000 * ((lookahead * dt) + currState.eTime), 2)) + 0.2 * (std::pow(controlInput - prevU, 2));
}


// Define the control function
double Controller::control(SystemState currState) {

	// Create a function pointer for our object function.
	std::function<double(std::vector<double>*)> p_ObjectFcn{ std::bind(&Controller::ObjectFcn, this, std::placeholders::_1, currState) };
	
    // initialize a gradient descent object
    gradDescent gd;
	
	// Assign the object function.
	gd.SetObjectFcn(p_ObjectFcn);
	
	// Call optimize.
    std::vector<double> startPoint = {prevU};
	
    // Optimize the control input
    std::vector<double> optimalVector = gd.Optimize(&startPoint, &currState.prevU);

    // Bound the optimal force
    double optimalForce = std::max(std::min(optimalVector[0], max), min);

    // Convert the force to an air brake input
    double optimalControl = forceToAirBrake(optimalForce);

    // Ensure the control input is within the specified range
    return optimalControl;
}