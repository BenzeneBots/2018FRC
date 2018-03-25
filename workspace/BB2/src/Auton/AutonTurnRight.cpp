/*
 * AutonTurnRight.cpp
 *
 *  Created on: Mar 10, 2018
 *      Author: Murali
 */

#include <Auton/AutonTurnRight.h>


#define TURN_P_CONST 0.0072

AutonTurnRight::AutonTurnRight(Drive *robotDrive, double angle) {
	drive = robotDrive;
	targetAngle = angle;
	turnState = turning;

	leftSpeed = 0.0;
	rightSpeed = 0.0;
}

AutonTurnRight::~AutonTurnRight() {
	// TODO Auto-generated destructor stub
}

void AutonTurnRight::Initialize(){
	drive->ResetYaw();
	turnState = turning;
}

bool AutonTurnRight::Run(){
	double currentYaw = drive->GetYaw();
	double angleError =  fabs(targetAngle) - fabs(currentYaw);
	leftSpeed = TURN_P_CONST * angleError;
	rightSpeed = -1.0  * TURN_P_CONST * angleError;

	printf("Yaw: %f\n", currentYaw);


	if(fabs(angleError) <= 1.0){
		drive->TankDrive(0.0,0.0);
		drive->ResetYaw();
		drive->ResetEncoders();
		printf("Finished!\n");
		return true;
	}
	drive->TankDrive(leftSpeed, rightSpeed);

	return false;
}
