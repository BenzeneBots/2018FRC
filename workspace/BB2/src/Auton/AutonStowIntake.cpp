/*
 * AutonStowIntake.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#include <Auton/AutonStowIntake.h>
#include <WPILib.h>

#define INTAKE_STOW_TIME 2.0 //retune if needed


AutonStowIntake::AutonStowIntake(Intake *robotIntake) {
	intake = robotIntake;
	SetTimeout(INTAKE_STOW_TIME);
	autonTimer = new Timer();
}

AutonStowIntake::~AutonStowIntake() {
	// TODO Auto-generated destructor stub
}

void AutonStowIntake::Initialize(){
	autonTimer->Reset();
	autonTimer->Start();
}

bool AutonStowIntake::Run(){
	printf("Deploying \n");
	intake->StowIntake();
	if(autonTimer->Get() >= 1.5){
		autonTimer->Stop();
		return true;
	}
	return false;
}

