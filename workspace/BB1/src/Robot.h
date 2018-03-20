/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <SmartDashboard/SendableChooser.h>
#include <TimedRobot.h>

#define MAX_V		2785		// Max Native Units Velocity As Measured

const uint8_t kTO = 0;				// No timeout, return right away without error code.
const uint8_t kTimeoutMs = 10;		// Max 10ms for timeouts.
const uint8_t kPIDLoopIdx = 0;

struct gains {
	double ff = 1023.0 / MAX_V;	// Feed-Forward = 1023 Max Velocity / Max Velocity Native Units (as measured).
	double p = 0.3;			// Proportional ~= (10% * 1023) / (350nu worst err measured).
	double i = 0.002;		// Integral ~= 1/100th of p.
	double d = 30.0;		// Derivative ~= 10x p.
	double iZone = 300;		// Clear integral if error is above this threshold.
	double peakOut = 1.0;	// Maximum motor output allowed.
};

typedef gains gains;

gains mpGains;	// PID Gains for Motion Profiling.


