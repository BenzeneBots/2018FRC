/*
 * AutonOuttake.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#include <Auton/AutonOuttake.h>

AutonOuttake::AutonOuttake(Intake *robotIntake, double timeout) {
	intake = robotIntake;
	timeoutTime = timeout;
	outtakeTimer = new Timer();

}

AutonOuttake::~AutonOuttake() {
	// TODO Auto-generated destructor stub
}

void AutonOuttake::Initialize(){
	outtakeTimer->Reset();
	outtakeTimer->Start();

}

bool AutonOuttake::Run(){
	if(outtakeTimer->Get() >= timeoutTime){
		intake->StopIntake();
		outtakeTimer->Stop();
		return true;
	}

	intake->OuttakeCubes();
	return false;

}
