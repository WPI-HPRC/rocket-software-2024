#pragma once

#include <Arduino.h>

// IREC 2024 Rocket Global Definitions (4/16/2024)

constexpr static float magneticDip = -11.9 * (180/PI); // [rad] Magnetic Inclination of launch site
constexpr static float rocketMass = 22.745; // [kg] Rocket mass from ORK
constexpr static float C_d = 0.5; // Eyeball averaged from ORK
constexpr static float S_r = (PI/4) * (0.1524*0.1524) + (0.00088386*4); // [m^2] Cross Sectional Area -- Body Tube + 4 Fins

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
#define MOTOR_BURN_TIME 3.1 * 1000.0

// Acceleration threshold for burnout detection, in G's
// checking if average Z acceleration is less than 0.3 G's
#define BURNOUT_THRESHOLD 0.3

// Coast -------------------------
// Coast to DrogueDescent Conditions
    // average vertical velocity <= 0
// Coast to Abort Conditions
    // time in Coast > 1.5 * TIME_IN_COAST

// seconds, OpenRocket for Test Launch 2/17
#define TIME_IN_COAST 18.4 * 1000.0

// DrogueDescent -------------------------
// DrogueDescent to MainDescent Conditions
    // average vertical velocity <= MAIN_DEPLOY_VELOCITY
// DrogueDescent to Abort Conditions
    // time in DrogueDescent > 1.2 * TIME_IN_DROGUE_DESCENT

// converts ft/s to m/s, OpenRocket sim Test Launch 2/17
#define FPS_TO_MPS 3.281

// Given in FPS from OpenRocket, convert to m/s
// checking if average vertical velocity is less than or equal to 82.6 ft/s
#define MAIN_DEPLOY_VELOCITY 82.6 / FPS_TO_MPS

// seconds, OpenRocket for Test Launch 2/17
#define TIME_IN_DROGUE_DESCENT 67.6 * 1000.0

// MainDescent -------------------------
// MainDescent to Recovery Conditions
    // average vertical velocity < LANDING_VELOCITY
// MainDescent to Abort Conditions
    // time in MainDescent > 1.1 * TIME_IN_MAIN_DESCENT

// 88 seconds, OpenRocket for Test Launch 2/17
#define TIME_IN_MAIN_DESCENT 90.0 * 1000.0

// Upper bound for landing velocity
// checking if average vertical velocity is less than 5 m/s
#define LANDING_VELOCITY 5.0

// Recovery -------------------------

// Abort -------------------------