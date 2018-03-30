/*
 * AutonTurnLeft.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#include <Auton/AutonTurnLeft.h>

//Drive Constants
#define RIGHT_DRIVE_CORRECTION 1.0

//Auton Left Turning Ramping Constants
#define MIN_LEFT_TURN_DRIVE_SPEED 0.26
#define MIN_LEFT_TURN_ANG_DIFFERENCE 20.0
#define STEP1_LEFT_TURN_ANG_DIFFERENCE 32.0
#define STEP2_LEFT_TURN_ANG_DIFFERENCE 48.0
//#define LEFT_TURN_RAMP_RATE 0.55
#define STEP1_LEFT_TURN_DRIVE_SPEED 0.30
#define STEP2_LEFT_TURN_DRIVE_SPEED 0.35
#define MAX_LEFT_TURN_DRIVE_SPEED 0.45

AutonTurnLeft::AutonTurnLeft(Drive *robotDrive, double angle) {
	drive = robotDrive;
	turnAngle = angle;
	startAngle = 0;
	targetAngle = 0;
}

AutonTurnLeft::~AutonTurnLeft() {
	// TODO Auto-generated destructor stub
}

void AutonTurnLeft::Initialize(){
	startAngle = drive->GetYaw();
	targetAngle = turnAngle + startAngle;
}

bool AutonTurnLeft::Run(){
	double currentAngle = drive->GetYaw();
	double angDifference = targetAngle - currentAngle;

	if(currentAngle >= targetAngle){
		drive->TankDrive(0.0,0.0);
		drive->ResetEncoders();
		return true;
	}
	else{
		printf("Current Angle %f \n", currentAngle);
		printf("startYaw %f \n", startAngle);

		double adjTurnSpeed= drive->AutonRamping2(angDifference,MIN_LEFT_TURN_DRIVE_SPEED, STEP1_LEFT_TURN_DRIVE_SPEED,STEP2_LEFT_TURN_DRIVE_SPEED,MAX_LEFT_TURN_DRIVE_SPEED,MIN_LEFT_TURN_ANG_DIFFERENCE,STEP1_LEFT_TURN_ANG_DIFFERENCE,STEP2_LEFT_TURN_ANG_DIFFERENCE);
		drive->TankDrive(-1.0 *adjTurnSpeed,adjTurnSpeed *RIGHT_DRIVE_CORRECTION);
		return false;
	}
}
