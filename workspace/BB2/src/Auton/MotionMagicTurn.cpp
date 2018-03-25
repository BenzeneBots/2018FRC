/*
 * MotionMagicTurn.cpp
 *
 *  Created on: Mar 17, 2018
 *      Author: Sanket Nayak
 */

#include <Auton/MotionMagicTurn.h>

MotionMagicTurn::MotionMagicTurn(Drive* drivetrain, double angle,bool isRightTurn) {
	drive = drivetrain;
	ang = angle;
	initTimer = new Timer();
	correctionAngle = 0;
	targetAngle = 0;
	//true if right, false if left
	side = isRightTurn;
}
MotionMagicTurn::~MotionMagicTurn(){
}
void MotionMagicTurn::Initialize(){
	correctionAngle = drive->GetYaw();
	drive->ResetEncoders();
	initTimer->Reset();
	initTimer->Start();
	drive->NeutralizeDrive();
	drive->ResetYaw();
}
bool MotionMagicTurn::Run(){
	if(side){
		targetAngle = correctionAngle + ang;
	}else{
		targetAngle = (-1.0 * correctionAngle) + ang;
	}

	drive->MotionMagicTurn(targetAngle);

	if((fabs(drive->GetAverageVelocity())<=1)&&(initTimer->Get()>=0.3)){
		initTimer->Stop();
		drive->ResetFusedHeading();
		printf("Done Turning! \n");
		return true;
	}else{
		return false;
		printf("Turning... \n");
	}
}
