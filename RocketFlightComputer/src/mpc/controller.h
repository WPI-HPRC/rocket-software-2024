#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <functional>

// Define SystemState structure
struct SystemState {
    double x, y, z, vx, vy, vz, theta, phi, psi, prevU, eTime;
        // x: x-position
        // y: y-position
        // z: z-position
        // vx: x-velocity
        // vy: y-velocity
        // vz: z-velocity
        // theta: pitch
        // phi: roll
        // psi: yaw
        // prevU: control input at the previous timestep
        // eTime: elapsed time
};

// Define Controller class
class Controller {
private:
    double lookahead; // how many time steps to look ahead
    double dt; // timestep
    double max; // maximum control input (in N) 
    double min; // minimum control input (in N) 
    double prevU; // control input at the previous timestep (in N)
    double mass; // mass of the rocket (in kg)

    // Define the forceToAirBrake function
    double forceToAirBrake(double force);

    // Define the ObjectFcn function to perform the optimization over 
    double ObjectFcn(std::vector<double> *funcLoc, SystemState currState);


public:
    // Constructor
    Controller(double lookahead, double dt, double max, double min, double prevU, double mass)
        : lookahead(lookahead), dt(dt), max(max), min(min), prevU(prevU), mass(mass) {}
        // lookahead: how many time steps to look ahead
        // dt: timestep
        // max: maximum control input (in N)
        // min: minimum control input (in N)
        // prevU: control input at the previous timestep (in N)

    // Define the control function
    double control(SystemState currState);
};