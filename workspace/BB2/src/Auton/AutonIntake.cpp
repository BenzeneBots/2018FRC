/*
 * AutonIntake.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#include <Auton/AutonIntake.h>
#include <WPILib.h>

AutonIntake::AutonIntake(Intake *robotIntake, double timeout) {
	intake = robotIntake;
	timeoutTime = timeout;
	intakeTimer = new Timer();
}

AutonIntake::~AutonIntake() {
	// TODO Auto-generated destructor stub
}

void AutonIntake::Initialize(){
intakeTimer->Reset();
intakeTimer->Start();
}

bool AutonIntake::Run(){
	if(intakeTimer->Get() >= timeoutTime){
		intake->StopIntake();
		intakeTimer->Stop();
		return true;
	}

	intake->IntakeCubes();
	return false;
}
