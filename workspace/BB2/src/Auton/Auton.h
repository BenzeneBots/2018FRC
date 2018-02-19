/*
 * Auton.h
 *
 *  Created on: Feb 18, 2018
 *      Author: Sanket Nayak
 */

#ifndef SRC_AUTON_AUTON_H_
#define SRC_AUTON_AUTON_H_

#include <ctre/Phoenix.h>
#include <WPILib.h>
#include <Subsystems/Drive.h>
#include <Subsystems/Elevator.h>
#include <Subsystems/Intake.h>

class Auton {
public:
	Auton(int);
	bool AutonDriveStraight(double,Drive*);
	bool AutonTurnRight(double,Drive*);
	bool AutonTurnLeft(double,Drive*);
	bool AutonLowerElevator(Elevator*);
	bool AutonLiftToSwitch(double,Elevator*);
	bool AutonLiftToScale(double,Elevator*);
	bool AutonOuttakeCube(Intake*);
private:
	PigeonIMU *_pidgey;
};

#endif /* SRC_AUTON_AUTON_H_ */
