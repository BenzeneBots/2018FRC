/*
 * Drive.cpp
 *
 *  Created on: Feb 26, 2018
 *      Author: Sanket Nayak
 */

#include <ctre/Phoenix.h>
#include <WPILib.h>
#include <Drive.h>

Drive::Drive(int frontLeftPort,int backLeftPort, int frontRightPort, int backRightPort) {

	//talon code
	frontLeft = new WPI_TalonSRX(frontLeftPort);
	backLeft = new WPI_TalonSRX(backLeftPort);
	frontRight = new WPI_TalonSRX(frontRightPort);
	backRight = new WPI_TalonSRX(backRightPort);

	backLeft->Set(ControlMode::Follower, frontLeft->GetDeviceID());
	backRight->Set(ControlMode::Follower, frontRight->GetDeviceID());

	drivetrain = new DifferentialDrive(*frontLeft, *frontRight);
}
void Drive::TankDrive(double left, double right){
	drivetrain->TankDrive(left, right, false);
}
