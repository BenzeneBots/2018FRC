/*
 * AutoDeployIntake.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#include <Auton/AutoDeployIntake.h>

#define INTAKE_STOW_TIME 2.0 //retune if needed


AutoDeployIntake::AutoDeployIntake(Intake *robotIntake) {
	intake = robotIntake;
	SetTimeout(INTAKE_STOW_TIME);
}

AutoDeployIntake::~AutoDeployIntake() {
	// TODO Auto-generated destructor stub
}

void AutoDeployIntake::Initialize(){}

bool AutoDeployIntake::Run(){
	intake->DeployIntake();
	if(IsTimeoutExpired()) return true;
	return false;
}

