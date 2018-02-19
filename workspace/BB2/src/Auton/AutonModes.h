/*
 * AutonModes.h
 *
 *  Created on: Feb 19, 2018
 *      Author: Sanket Nayak
 */
#include <ctre/Phoenix.h>
#include <WPILib.h>

#ifndef SRC_AUTON_AUTONMODES_H_
#define SRC_AUTON_AUTONMODES_H_

#include <Auton/AutonTasks.h>
#include <Subsystems/Drive.h>
#include <Subsystems/Elevator.h>
#include <Subsystems/Intake.h>

class AutonModes {
public:
	AutonModes();
	void DriveStraight(double,Drive*,Auton*);
	void Center1CubeLeftSwitch(double,double,Drive*,Auton*);
	void Center1CubeRightSwitch(Auton*);
	void Center1CubeLeftScale(Auton*);
	void Center1CubeRightScale(Auton*);
};

#endif /* SRC_AUTON_AUTONMODES_H_ */
