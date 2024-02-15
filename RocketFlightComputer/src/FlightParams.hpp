#pragma once
// These constants define transitions between states and the conditions for those transitions

// PreLaunch -------------------------

// measured in G's
// checking if average Z acceleration is greater than 4 G's
#define LAUNCH_ACCEL_THRESHOLD 4.0


// Launch -------------------------

// Acceleration threshold for burnout detection, in G's
// checking if average Z acceleration is less than 0.3 G's
#define BURNOUT_THRESHOLD 0.3

// 3 second timeout, as defined in OpenRocket for Test Launch 2/17
#define MOTOR_BURN_TIME 3 * 1000


// Coast -------------------------

// seconds, OpenRocket for Test Launch 2/17
#define TIME_IN_COAST 19 * 1000


// DrogueDescent -------------------------

// converts ft/s to m/s, OpenRocket sim Test Launch 2/17
#define FPS_TO_MPS 3.281

// Given in FPS from OpenRocket, convert to m/s
// checking if average vertical velocity is less than or equal to 84.7 ft/s
#define MAIN_DEPLOY_VELOCITY 84.7 / FPS_TO_MPS

// seconds, OpenRocket for Test Launch 2/17
#define TIME_IN_DROGUE_DESCENT 71 * 1000


// MainDescent -------------------------

// 88 seconds, OpenRocket for Test Launch 2/17
#define TIME_IN_MAIN_DESCENT 88 * 1000

// Upper bound for landing velocity
// checking if average vertical velocity is less than 5 m/s
#define LANDING_VELOCITY 5.0


// Recovery -------------------------

// Abort -------------------------