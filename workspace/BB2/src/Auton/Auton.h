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

#define AUTON_DRIVE_SPEED 0.5
#define LEFT_DRIVE_CORRECTION 1.0
#define INCHES_PER_TICK 0.00460194335

class Auton {
public:
	Auton();
	void AutonDriveStraight(double,Drive*);
};

#endif /* SRC_AUTON_AUTON_H_ */
