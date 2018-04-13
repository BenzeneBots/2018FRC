/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#ifndef Robot_H_
#define Robot_H_

#include <string>

#include <SmartDashboard/SendableChooser.h>
#include <TimedRobot.h>

// Set true to enable the compressor.
#define COMPRESSOR	true

// UnComment for Practice Bot
//#define PRACTICE_BOT

#define NUM_PATHS	9	// There are six possible paths in Auto mode.

#define XBOX
#define NUM_BTN		11			// Number of buttons on a Joystick.

enum AutonPathId {
	CenterLeftSwitch, CenterRightSwitch, LeftSideSwitch, RightSideSwitch,
	LeftNearScale, RightNearScale, LeftFarScale, RightFarScale, DriveStraight,
	TestFunction, unknownPathID
};
typedef AutonPathId AutonPathId;

// Store all the buttons from all the joysticks in this one structure.
struct btns {
	bool btn[ NUM_BTN ], btn2[ NUM_BTN ];				// Current Button State
	bool btnPress[ NUM_BTN ], btnPress2[ NUM_BTN ];		// True on button press.
	int pov, pov2;										// Current POV value.
};

// Primary Joystick Button Defs
enum joyBtn { unknown = -1,
	reverse = 1, strait = 2, clawClose = 3, clawOpen = 4, five = 5,
	revDir = 6, intake = 7, tankDrive = 8, nine = 9,
	ten = 10, eleven = 11, climber = 12
};

// 1 = Outtake
// 2 = Drive Strait
// 6 = Reverse Dir
// 7 = Intake
// 8 = TankDrive( 1, 1 )


// Second Joystick Button Defs
enum joy2Btn { unknown2 = -1,
	intake2 = 1, eject2 = 2, three2 = 3, four2 = 4, clawOpen2 = 5,
	clawClose2 = 6, eScale2 = 7, eight2 = 8, eSwitch2 = 9,
	ten2 = 10, eHome2 = 11, climber2 = 12
};

// 1 = Intake
// 2 = Outtake
// 5 = Open Claw
// 6 = Close Claw
// 7 = Scale Height
// 9 = Switch Height
// 11 = Home Height
// 12 = Climber
// 8, 10, 12 = Switch back to elevator joystick control.
// POV = 180 -> Pickup Claw
// POV = 0 -> Deploy Claw


// All the possible directions the POV hat can be pushed.
enum pov { 
	povNone = -1,
	povN = 0, 
	povNE = 45, 
	povE = 90, 
	povSE = 135, 
	povS = 180, 
	povSW = 225, 
	povW = 270, 
	povNW = 315
};
enum elevatorHeight{
	Ground,
	SwitchHeight,
	ScaleHeight
};


//#define MAX_V		2785			// Max Native Units Velocity As Measured

const uint8_t kTO = 0;				// No timeout, return right away without error code.
const uint8_t kTimeoutMs = 10;		// Max 10ms for timeouts.
const uint8_t kPIDLoopIdx = 0;

struct gains {
	double ff = 0.3673;		// Feed-Forward = 1023 Max Velocity / Max Velocity Native Units (as measured).
	double p = 0.3;			// Proportional ~= (10% * 1023) / (350nu worst err measured).
	double i = 0.002;		// Integral ~= 1/100th of p.
	double d = 30.0;		// Derivative ~= 10x p.
	double iZone = 300;		// Clear integral if error is above this threshold.
	double peakOut = 1.0;	// Maximum motor output allowed.
};

typedef gains gains;

// Each waypoint group has a name defined in this enum.
enum paths {
	Side_Switch=0, Side_Scale=1, Side_SwitchFar=2,
	Side_ScaleFar=3, Mid_SwitchLeft=4, 	Mid_SwitchRight=5, Left_SwitchMid=6,
	Right_SwitchMid=7, Switch_Cube=8,
	NA=100 };
typedef paths paths;

enum startPos { left, mid, right };
typedef startPos startPos;

#endif	// Robot_H_


