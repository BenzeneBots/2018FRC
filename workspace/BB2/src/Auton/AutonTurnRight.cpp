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
	targetAngle = -1.0 * angle;
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
	double angleError =  fabs(currentYaw) - fabs(targetAngle);
	leftSpeed = TURN_P_CONST * angleError;
	rightSpeed = -1.0  * TURN_P_CONST * angleError;

	printf("Yaw: %f\n", currentYaw);


	if(fabs(angleError) <= 1.0){
		drive->TankDrive(0.0,0.0);

	}else{
	drive->TankDrive(leftSpeed, rightSpeed);
	}

	if((fabs(drive->GetLeftVelocity()) <= 1) && (turnTimer->Get() >= 0.3)){
		drive->ResetYaw();
		drive->ResetEncoders();
		drive->TankDrive(0.0,0.0);
		turnTimer->Stop();
		printf("Done Turning! \n");
		return true;
	}
	return false;

}
