/*
 *	BB1 - Benzene Bots FRC 2018 Robot Source Code
 *
 *
 */
#pragma once

#include <string>

#include <SmartDashboard/SendableChooser.h>
#include <TimedRobot.h>

// Used in ConfigPeakOutputForward() to limit drivetrain motors speed
// in all configurations.
#define MAX_SPEED	1.0			// Limit Drive Speed to X%.

#define FF_MAX		2785		// Max Native Units As Measured

const uint8_t kTimeoutMs = 10;		// 10ms for timeouts.
const uint8_t kTO = 10;
const uint8_t kPIDLoopIdx = 0;

const static int REMOTE_0 = 0;
const static int REMOTE_1 = 1;
const static int PID_PRIMARY = 0;
const static int PID_TURN = 1;
const static int SLOT_0 = 0;
const static int SLOT_1 = 1;
const static int SLOT_2 = 2;
const static int SLOT_3 = 3;

struct Gains {
	double kP, kI, kD, kF;
	double kIzone;
	double kPeakOutput;
};

//                                         kP   kI   kD   kF              Iz    PeakOut
constexpr static Gains kGains_Distanc = { 0.1, 0.0,  0.0, 0.0,            100,  0.50 };
constexpr static Gains kGains_Turning = { 2.0, 0.0,  4.0, 0.0,            200,  1.00 };
constexpr static Gains kGains_Velocit = { 0.1, 0.0, 20.0, 1023.0/FF_MAX,  300,  0.50 }; // measured 3200 max velocity
constexpr static Gains kGains_MotProf = { 1.0, 0.0,  0.0, 1023.0/FF_MAX,  400,  1.00 }; // measured 3200 max velocity
constexpr static Gains kGains_MM =      { 0.4, 0.001, 4.0, 1023.0/FF_MAX,  400,  1.00 }; // measured 3200 max velocity

const static int kSlot_Distanc = SLOT_0;
const static int kSlot_Turning = SLOT_1;
const static int kSlot_Velocit = SLOT_2;
const static int kSlot_MotProf = SLOT_3;




