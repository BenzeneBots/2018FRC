/*
 * AutonCloseClaw.cpp
 *
 *  Created on: Mar 29, 2018
 *      Author: Sabita Dhal
 */

#include <Auton/AutonCloseClaw.h>

AutonCloseClaw::AutonCloseClaw(Intake *robotIntake) {
	intake = robotIntake;
}

AutonCloseClaw::~AutonCloseClaw() {
	// TODO Auto-generated destructor stub
}

void AutonCloseClaw::Initialize(){}

bool AutonCloseClaw::Run(){
	intake->CloseClaw();
	return true;
}
