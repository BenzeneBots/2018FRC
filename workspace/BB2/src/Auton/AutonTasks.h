/*
 * Auton.h
 *
 *  Created on: Feb 18, 2018
 *      Author: Sanket Nayak
 */

#ifndef SRC_AUTON_AUTONTASKS_H_
#define SRC_AUTON_AUTONTASKS_H_

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
	bool AutonLiftToSwitch(Elevator*);
	bool AutonLiftToScale(Elevator*);
	bool AutonOuttake(Intake*);
	bool AutonDeployIntake(Intake*);
	bool Reset(Drive*);

private:
	PigeonIMU *_pidgey;
	enum Steps{Straight1,Turn1,Straight2,Turn2,Straight3,Deploy,Score};

};

#endif /* SRC_AUTON_AUTONTASKS_H_ */
