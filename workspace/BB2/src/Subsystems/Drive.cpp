/*
 * Drive.cpp
 *
 *  Created on: Jan 23, 2018
 *      Author: sabitadhal
 */

#include <Subsystems/Drive.h>
#include <WPILib.h>
#include <ctre/Phoenix.h>

DifferentialDrive *drivetrain;

Drive::Drive(Victor left1, Victor left2, Victor right1, Victor right2) {
	SpeedControllerGroup *leftDrive = new SpeedControllerGroup(left1, left2);
	SpeedControllerGroup *rightDrive = new SpeedControllerGroup(right1, right2);

    drivetrain = new DifferentialDrive(leftDrive, rightDrive);
	// TODO Auto-generated constructor stub

}

Drive::~Drive() {
	// TODO Auto-generated destructor stub
}

void ArcadeDrive(double speed, double turn){
	drivetrain->ArcadeDrive(speed, turn, false);
}
