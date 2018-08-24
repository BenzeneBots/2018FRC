/*
 * MotionMagicStraight.cpp
 *
 *  Created on: Mar 17, 2018
 *      Author: Sanket Nayak
 */

#include <Auton/MotionMagicStraight.h>

MotionMagicStraight::MotionMagicStraight(Drive* drivetrain, double distance) {
	drive = drivetrain;
	dist = distance;
	initTimer = new Timer();
}
MotionMagicStraight::~MotionMagicStraight(){

}
void MotionMagicStraight::Initialize(){
	drive->ResetEncoders();
	initTimer->Reset();
	initTimer->Start();
	drive->NeutralizeDrive();
}
bool MotionMagicStraight::Run(){
	drive->MotionMagicStraight(dist);

	if((fabs(drive->GetAverageVelocity())<=1)&&(initTimer->Get()>=0.5)){
		initTimer->Stop();
		drive->TankDrive(0.0,0.0);
		drive->ResetYaw();
		printf("Done Driving! \n");
		return true;
	}else{
		return false;
		printf("Driving... \n");
	}
}
