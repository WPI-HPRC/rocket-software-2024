#pragma once

// Debug things
// #define DEBUG_MODE
#ifdef DEBUG_MODE
#define NO_TRANSITION
#define NO_FLASH
#define NO_XBEE
#define SERIAL_TELEMETRY
#define WAIT_FOR_SERIAL
#endif

// These constants define transitions between states and the conditions for those transitions

// PreLaunch -------------------------
// PreLaunch to Launch Conditions
    // average Z acceleration > LAUNCH_ACCEL_THRESHOLD
// Cannot go to Abort state from PreLaunch

// measured in G's
// checking if average Z acceleration is greater than 4 G's
#define LAUNCH_ACCEL_THRESHOLD 4.0

// Launch -------------------------
// Launch to Coast Conditions
    // time in Launch >= MOTOR_BURN_TIME
    // and
    // average Z acceleration < BURNOUT_THRESHOLD
// Launch to Abort Conditions
    // time in Launch > 2 * MOTOR_BURN_TIME

// 3.1 second timeout, as defined in OpenRocket for Test Launch 2/17
constexpr float MOTOR_BURN_TIME = 3.1 * 1000.0;

// Acceleration threshold for burnout detection, in G's
// checking if average Z acceleration is less than 0.3 G's
#define BURNOUT_THRESHOLD 0.3

// Coast -------------------------
// Coast to DrogueDescent Conditions
    // average vertical velocity <= 0
// Coast to Abort Conditions
    // time in Coast > 1.5 * TIME_IN_COAST

// Angle in degrees to change the airbrake servo by every COAST_AIRBRAKE_INCREMENT_LOOPS loops while in sweep mode
#define COAST_AIRBRAKE_INCREMENT_ANGLE 10

// Number of loops to wait before incrementing the airbrake servo angle while in sweep mode
// Each loop is about 25ms
#define COAST_AIRBRAKE_INCREMENT_LOOPS 4

#define AIRBRAKE_FULL_EXTENSION 1775
#define AIRBRAKE_75_EXTENSION 1734
#define AIRBRAKE_HALF_EXTENSION 1693
#define AIRBRAKE_25_EXTENSION 1651
#define AIRBRAKE_RETRACTED 1500
// seconds, OpenRocket for Test Launch 2/17
constexpr float TIME_IN_COAST = 18.4 * 1000.0;

// DrogueDescent -------------------------
// DrogueDescent to MainDescent Conditions
    // average vertical velocity <= MAIN_DEPLOY_VELOCITY
// DrogueDescent to Abort Conditions
    // time in DrogueDescent > 1.2 * TIME_IN_DROGUE_DESCENT

// converts ft/s to m/s, OpenRocket sim Test Launch 2/17
#define FPS_TO_MPS 3.281

// Given in FPS from OpenRocket, convert to m/s
// checking if average vertical velocity is less than or equal to 82.6 ft/s
constexpr float MAIN_DEPLOY_VELOCITY = 82.6 / FPS_TO_MPS;

// seconds, OpenRocket for Test Launch 2/17
constexpr float TIME_IN_DROGUE_DESCENT = 67.6 * 1000.0;

// MainDescent -------------------------
// MainDescent to Recovery Conditions
    // average vertical velocity < LANDING_VELOCITY
// MainDescent to Abort Conditions
    // time in MainDescent > 1.1 * TIME_IN_MAIN_DESCENT

// 88 seconds, OpenRocket for Test Launch 2/17
constexpr float TIME_IN_MAIN_DESCENT = 90.0 * 1000.0;

// Upper bound for landing velocity
// checking if average vertical velocity is less than 5 m/s
#define LANDING_VELOCITY 5.0

// Recovery -------------------------

// Abort -------------------------
