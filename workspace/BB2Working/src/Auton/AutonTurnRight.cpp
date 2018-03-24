/*
 * AutonTurnRight.cpp
 *
 *  Created on: Mar 10, 2018
 *      Author: Murali
 */

#include <Auton/AutonTurnRight.h>


#define RIGHT_DRIVE_CORRECTION 1.0 //TODO tune this

//Auton Right Turning Ramping Constants
#define MIN_RIGHT_TURN_DRIVE_SPEED 0.26
#define MIN_RIGHT_TURN_ANG_DIFFERENCE 20.0
#define STEP1_RIGHT_TURN_ANG_DIFFERENCE 32.0
#define STEP2_RIGHT_TURN_ANG_DIFFERENCE 48.0
//#define RIGHT_TURN_RAMP_RATE 0.55
#define STEP1_RIGHT_TURN_DRIVE_SPEED 0.30
#define STEP2_RIGHT_TURN_DRIVE_SPEED 0.35
#define MAX_RIGHT_TURN_DRIVE_SPEED 0.45

AutonTurnRight::AutonTurnRight(Drive *robotDrive, double angle) {
	drive = robotDrive;
	turnAngle = angle;
	startAngle = 0;
	targetAngle = 0;
}

AutonTurnRight::~AutonTurnRight() {
	// TODO Auto-generated destructor stub
}

void AutonTurnRight::Initialize(){
	startAngle = drive->GetYaw();
	targetAngle = startAngle - turnAngle;
}

bool AutonTurnRight::Run(){
	double currentAngle = drive->GetYaw();
	double angDifference = currentAngle - targetAngle;

	if(currentAngle <= targetAngle){
		drive->TankDrive(0.0,0.0);
		drive->ResetEncoders();
		return true;
	}
	else{
		printf("Current Angle %f \n", currentAngle);
		printf("Target Angle %f \n", targetAngle);
		printf("startYaw %f \n", startAngle);

		double adjTurnSpeed= drive->AutonRamping2(angDifference,MIN_RIGHT_TURN_DRIVE_SPEED, STEP1_RIGHT_TURN_DRIVE_SPEED,STEP2_RIGHT_TURN_DRIVE_SPEED,MAX_RIGHT_TURN_DRIVE_SPEED,MIN_RIGHT_TURN_ANG_DIFFERENCE,STEP1_RIGHT_TURN_ANG_DIFFERENCE,STEP2_RIGHT_TURN_ANG_DIFFERENCE);
		drive->TankDrive(adjTurnSpeed, -1.0 * adjTurnSpeed *RIGHT_DRIVE_CORRECTION);
	}
	return false;
}
