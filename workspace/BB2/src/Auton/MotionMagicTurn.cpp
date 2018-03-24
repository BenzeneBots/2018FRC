/*
 * MotionMagicTurn.cpp
 *
 *  Created on: Mar 17, 2018
 *      Author: Sanket Nayak
 */

#include <Auton/MotionMagicTurn.h>

MotionMagicTurn::MotionMagicTurn(Drive* drivetrain, double angle) {
	drive = drivetrain;
	ang = angle;
	initTimer = new Timer();
	correctionAngle = 0;
	targetAngle = 0;
}
MotionMagicTurn::~MotionMagicTurn(){
}
void MotionMagicTurn::Initialize(){
	correctionAngle = drive->GetFusedHeading();
	drive->ResetEncoders();
	initTimer->Reset();
	initTimer->Start();
	drive->NeutralizeDrive();
	drive->ResetFusedHeading();
}
bool MotionMagicTurn::Run(){
	//assuming that turning right is positive
	targetAngle = correctionAngle + ang;
	drive->MotionMagicTurn(targetAngle);

	if((fabs(drive->GetAverageVelocity())<=1)&&(initTimer->Get()>=2.0)){
		initTimer->Stop();
		drive->ResetFusedHeading();
		printf("Done Turning! \n");
		return true;
	}else{
		return false;
		printf("Turning... \n");
	}
}
