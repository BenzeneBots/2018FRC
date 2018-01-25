/*
 * Drive.cpp
 *
 *  Created on: Jan 24, 2018
 *      Author: Murali
 */

#include <Subsystems/Drive.h>
#include <ctre/Phoenix.h>
#include <WPILib.h>
#include <Talon.h>

Drive::Drive(int left1ID, int left2ID, int right1ID, int right2ID) {
	//Initialize motor controllers based on inputs
	left1 = new TalonSRX(left1ID);
	left2 = new TalonSRX(left2ID);
	right1 = new TalonSRX(right1ID);
	right2 = new TalonSRX(right2ID);

	left2->Set(ControlMode::Follower, left1ID);
	right2->Set(ControlMode::Follower, left2ID);

	drivetrain = new DifferentialDrive(left1, right1);

}

Drive::~Drive() {
	// TODO Auto-generated destructor stub
}

void ArcadeDrive(double speed, double turn){
	drivetrain->ArcadeDrive(speed, turn);
}
