/*
 * AutonTurnRight1.cpp
 *
 *  Created on: Mar 30, 2018
 *      Author: Sabita Dhal
 */

#include <Auton/AutonTurnRight1.h>
#include <Subsystems/Drive.h>
#include <WPILib.h>

#define ACCEPTABLE_ANG 3.5
#define MIN_TURN_SPEED 0.23
#define MAX_TURN_SPEED 0.5

AutonTurnRight1::AutonTurnRight1(Drive *robotDrive, double angle) {
	drive = robotDrive;
	targetAngle = angle;

	leftSpeed = 0.0;
	rightSpeed = 0.0;
	speed = 0;

	turnTimer = new Timer();
	doneFlag = true;
}

AutonTurnRight1::~AutonTurnRight1() {
	// TODO Auto-generated destructor stub
}

void AutonTurnRight1::Initialize(){
	drive->ResetYaw();
	turnTimer->Reset();
}

bool AutonTurnRight1::Run(){
	double currentYaw = drive->GetYaw();
	double angleError =fabs(currentYaw) - fabs(targetAngle);

	printf("Yaw %f \n", drive->GetYaw());
	printf("Speed %f \n", drive->GetYaw());


	if(angleError >= 0){
		speed = (MAX_TURN_SPEED-MIN_TURN_SPEED) * pow(angleError,2)/(pow(angleError,2)+1300) + MIN_TURN_SPEED;
	}else {
		speed = -1.0*((MAX_TURN_SPEED-MIN_TURN_SPEED) * pow(angleError,2)/(pow(angleError,2)+1300) + MIN_TURN_SPEED);
	}

	leftSpeed = speed;
	rightSpeed = -1.0 * speed;

	if(fabs(angleError) <= 1.0){
		drive->TankDrive(0.0,0.0);
	}
	drive->TankDrive(leftSpeed, rightSpeed);

	if(fabs(angleError) <= ACCEPTABLE_ANG){
		drive->TankDrive(0.0,0.0);
		printf("Done Turning! \n");
	}else{
	drive->TankDrive(leftSpeed, rightSpeed);
	}

	if(fabs(drive->GetLeftVelocity()) <= .3){
		if(doneFlag ){
		turnTimer->Start();
		doneFlag = false;
		}
	}else{
		doneFlag = true;
		turnTimer->Stop();
		turnTimer->Reset();
	}

	if(turnTimer->Get() >= 0.254){
		drive->ResetYaw();
		drive->ResetEncoders();
		drive->TankDrive(0.0,0.0);
		turnTimer->Stop();
		printf("Done turning! \n");
		return true;
	}else{
		return false;
	}

	/*	if((drive->GetLeftVelocity() <= 1) && (turnTimer->Get() >= 0.3)){
			drive->ResetYaw();
			drive->ResetEncoders();
			turnTimer->Stop();
			return true;
		}else{
		return false;
		}
	*/
}

