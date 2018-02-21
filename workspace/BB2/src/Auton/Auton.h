/*
 * Auton.h
 *
 *  Created on: Feb 19, 2018
 *      Author: Sanket Nayak
 */

#ifndef SRC_AUTON_AUTON_H_
#define SRC_AUTON_AUTON_H_

#include <ctre/Phoenix.h>
#include <WPILib.h>
#include <Subsystems/Drive.h>
#include <Subsystems/Elevator.h>
#include <Subsystems/Intake.h>

bool AutonDriveStraight(double, Drive*);
bool AutonTurnRight(double, Drive*);
bool AutonTurnLeft(double, Drive*);
bool AutonSetHeight(double,Elevator*);
bool AutonMoveToHeight(Elevator*);
bool AutonIntake(Intake*);
bool AutonOuttake(Intake*);
bool AutonStopIntake(Intake*);
bool AutonDeployIntake(Intake*);
bool AutonStowIntake(Intake*);
bool AutonScoreSwitch(Elevator*, Intake*);




#endif /* SRC_AUTON_AUTON_H_ */
