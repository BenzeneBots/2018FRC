/*
 * AutonOpenClaw.cpp
 *
 *  Created on: Mar 29, 2018
 *      Author: Sabita Dhal
 */

#include <Auton/AutonOpenClaw.h>

AutonOpenClaw::AutonOpenClaw(Intake *robotIntake) {
	intake = robotIntake;
}

AutonOpenClaw::~AutonOpenClaw() {
	// TODO Auto-generated destructor stub
}

void AutonOpenClaw::Initialize(){}

bool AutonOpenClaw::Run(){
	intake->OpenClaw();
	return true;
}
