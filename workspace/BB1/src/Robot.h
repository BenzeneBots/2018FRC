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

const uint8_t kTimeoutMs = 10;		// 10ms for timeouts.
const uint8_t kPIDLoopIdx = 0;

const double 	f = 0.3119,		// Feed-Forward = 1023 Max Velocity / 3280nu(Max Native Units / 100ms measured).
				p = 0.3,		// Proportional
				i = 0.0, 		// Integral
				d = 5.0;		// Derivative


