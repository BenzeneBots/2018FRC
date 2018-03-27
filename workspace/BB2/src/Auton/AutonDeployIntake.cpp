/*
 * AutonDeployIntake.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#include <Auton/AutonDeployIntake.h>

#define INTAKE_STOW_TIME 2.0 //retune if needed


AutonDeployIntake::AutonDeployIntake(Intake *robotIntake) {
	intake = robotIntake;
	SetTimeout(INTAKE_STOW_TIME);
}

AutonDeployIntake::~AutonDeployIntake() {
	// TODO Auto-generated destructor stub
}

void AutonDeployIntake::Initialize(){}

bool AutonDeployIntake::Run(){
	printf("Deploying \n");
	intake->DeployIntake();
	if(IsTimeoutExpired()) return true;
	return false;
}

