/*
 * AutonIntake.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#include <Auton/AutonIntake.h>

AutonIntake::AutonIntake(Intake *robotIntake, double timeout) {
	SetTimeout(timeout);
	intake = robotIntake;
	// TODO Auto-generated constructor stub

}

AutonIntake::~AutonIntake() {
	// TODO Auto-generated destructor stub
}

void AutonIntake::Initialize(){

}

bool AutonIntake::Run(){
	if(IsTimeoutExpired()){
		intake->StopIntake();
		return true;
	}

	intake->IntakeCubes();
	return false;
}
