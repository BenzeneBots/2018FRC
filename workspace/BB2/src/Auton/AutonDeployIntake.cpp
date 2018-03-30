/*
 * AutonDeployIntake.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#include <Auton/AutonDeployIntake.h>
#include <WPILib.h>

#define INTAKE_STOW_TIME 2.0 //retune if needed


AutonDeployIntake::AutonDeployIntake(Intake *robotIntake) {
	intake = robotIntake;
	SetTimeout(INTAKE_STOW_TIME);
	deployTimer = new Timer();
}

AutonDeployIntake::~AutonDeployIntake() {
	// TODO Auto-generated destructor stub
}

void AutonDeployIntake::Initialize(){
	deployTimer->Reset();
	deployTimer->Start();
}

bool AutonDeployIntake::Run(){
	printf("Deploying \n");
	intake->DeployIntake();
	if(deployTimer->Get() >= 1.5){
		deployTimer->Stop();
		return true;
	}
	return false;
}

