/*
 * AutonTurnRight.cpp
 *
 *  Created on: Mar 10, 2018
 *      Author: Murali
 */

#include <Auton/AutonTurnRight.h>
#include <Subsystems/Drive.h>


#define TURN_P_CONST 0.0074

AutonTurnRight::AutonTurnRight(Drive *robotDrive, double angle) {
	drive = robotDrive;
	targetAngle = angle;
	turnState = turning;

	leftSpeed = 0.0;
	rightSpeed = 0.0;

	turnTimer = new Timer();
}

AutonTurnRight::~AutonTurnRight() {
	// TODO Auto-generated destructor stub
}

void AutonTurnRight::Initialize(){
	drive->ResetYaw();
	turnState = turning;

	turnTimer->Reset();
	turnTimer->Start();
}

bool AutonTurnRight::Run(){
	double currentYaw = drive->GetYaw();
	double angleError = fabs(targetAngle) - fabs(currentYaw);
	leftSpeed = -1.0 * TURN_P_CONST * angleError;
	rightSpeed = TURN_P_CONST * angleError;

	if(fabs(angleError) <= 1.0){
		drive->TankDrive(0.0,0.0);
	}
	drive->TankDrive(leftSpeed, rightSpeed);

	if((drive->GetLeftVelocity() <= 1) && (turnTimer->Get() >= 0.3)){
			drive->ResetYaw();
			drive->ResetEncoders();
			turnTimer->Stop();
			return true;
		}else{
		return false;
		}
}
