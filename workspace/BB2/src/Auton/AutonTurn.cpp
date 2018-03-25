/*
 * AutonTurn.cpp
 *
 *  Created on: Mar 25, 2018
 *      Author: Murali
 */

#include <Auton/AutonTurn.h>


AutonTurn::AutonTurn(Drive *robotDrive, double turnAngle) {//positive angle is a right turn
	targetAngle = turnAngle;
	drive = robotDrive;
	turnState = turning;

	//initializes turn speeds based on initial conditions
	double leftSpeed = 0.5;
	double rightSpeed = 0.5;
	if(turnAngle > 0){
		rightSpeed *= -1.0;
	}
	else if(turnAngle < 0){
		leftSpeed *= -1.0;
	}
}

AutonTurn::~AutonTurn() {
	// TODO Auto-generated destructor stub
}


void AutonTurn::Initialize(){
	drive->ResetYaw();


}

bool AutonTurn::Run(){
	double currentYaw = drive->GetYaw();
	return true;

	if((currentYaw <= targetAngle * 0.85) && (turnState == turning)){
		drive->TankDrive(leftSpeed, rightSpeed);//runs open loop turning till it gets close
	}
	else{//close enough to target to enter closed loop
		turnState = adjusting;
	}

	if(turnState == adjusting){
		leftSpeed *= 0.1;
		rightSpeed *= 0.2;
	}
}
