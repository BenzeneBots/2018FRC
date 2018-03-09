/*
 * AutonOuttake.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#include <Auton/AutonOuttake.h>

AutonOuttake::AutonOuttake(Intake *robotIntake, double timeout) {
	SetTimeout(timeout);
	intake = robotIntake;
}

AutonOuttake::~AutonOuttake() {
	// TODO Auto-generated destructor stub
}

void AutonOuttake::Initialize(){

}

bool AutonOuttake::Run(){
	if(IsTimeoutExpired()){
		intake->StopIntake();
		return true;
	}

	intake->OuttakeCubes();
	return false;

}
