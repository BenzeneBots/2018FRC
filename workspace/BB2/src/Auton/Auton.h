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
void AutonTurnRight(double, Drive*);
void AutonTurnLeft(double, Drive*);
void AutonMoveToHeight(double, Elevator*);
void AutonIntake(Intake*);
void AutonOuttake(Intake*);
void AutonDeployIntake(Intake*);
void AutonStowIntake(Intake*);




#endif /* SRC_AUTON_AUTON_H_ */
