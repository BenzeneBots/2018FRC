/*
 * ElevateDrive.cpp
 *
 *  Created on: Mar 27, 2018
 *      Author: Sabita Dhal
 */

#include <Auton/ElevateDrive.h>

ElevateDrive::ElevateDrive(Drive* drivetrain, double distance,Elevator *robotElevator, double heightTarget){
	drive = drivetrain;
	dist = distance;
	initTimer = new Timer();
	elevator = robotElevator;
	targetHeight = heightTarget;
}

ElevateDrive::~ElevateDrive(){
}

void ElevateDrive::Initialize(){
	drive->ResetEncoders();
	initTimer->Reset();
	initTimer->Start();
	drive->NeutralizeDrive();
	elevator->SetElevatorTarget(targetHeight);
}

bool ElevateDrive::Run(){
	drive->MotionMagicStraight(dist);

	if((fabs(drive->GetAverageVelocity())<=1)&&(initTimer->Get()>=0.5)){
		initTimer->Stop();
		drive->TankDrive(0.0,0.0);
		drive->ResetYaw();
		printf("Done Driving! \n");
		return elevator->MoveElevator(0.0);
	}else{
		elevator->MoveElevator(0.0);
		return false;
		printf("Driving... \n");
	}
}

