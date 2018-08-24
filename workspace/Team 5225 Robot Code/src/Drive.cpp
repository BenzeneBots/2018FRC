/*
 * Drive.cpp
 *
 *  Created on: Jan 28, 2018
 *      Author: Murali
 */

#include <Drive.h>
#include <ctre/Phoenix.h>
#include <WPILib.h>
#include <Math.h>


Drive::Drive(int frontLeftPort,int backLeftPort, int frontRightPort, int backRightPort) {

    // Create all drive motors

	//talon code
	frontLeft = new WPI_TalonSRX(frontLeftPort);
	backLeft = new WPI_TalonSRX(backLeftPort);
	frontRight = new WPI_TalonSRX(frontRightPort);
	backRight = new WPI_TalonSRX(backRightPort);

	backLeft->Set(ControlMode::Follower, frontLeft->GetDeviceID());
	backRight->Set(ControlMode::Follower, frontRight->GetDeviceID());

	drivetrain = new DifferentialDrive(*frontLeft, *frontRight);
}

void Drive::ArcadeDrive(double speed, double turn){//Drives the drivetrain based on
	drivetrain->ArcadeDrive(speed, turn, false);
}

void Drive::TankDrive(double left, double right){
	drivetrain->TankDrive(left, right, false);
}


//*/
