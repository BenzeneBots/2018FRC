/*
 * AutonTurnLeft.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#include <Auton/AutonTurnLeft.h>

#define TURN_P_CONST 0.0068


AutonTurnLeft::AutonTurnLeft(Drive *robotDrive, double angle) {
	drive = robotDrive;
	targetAngle = angle;
	turnState = turning;

	leftSpeed = 0.0;
	rightSpeed = 0.0;
}

AutonTurnLeft::~AutonTurnLeft() {
	// TODO Auto-generated destructor stub
}

void AutonTurnLeft::Initialize(){
	drive->ResetYaw();
	turnState = turning;
}

bool AutonTurnLeft::Run(){

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

