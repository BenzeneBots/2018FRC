/*
 * AutonStowIntake.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#include <Auton/AutonStowIntake.h>

#define INTAKE_STOW_TIME 2.0 //retune if needed

AutonStowIntake::AutonStowIntake(Intake *robotIntake) {
	SetTimeout(INTAKE_STOW_TIME);
	intake = robotIntake;
}

AutonStowIntake::~AutonStowIntake() {

}

void AutonStowIntake::Initialize(){}

bool AutonStowIntake::Run(){
	intake->StowIntake();
	if(IsTimeoutExpired()) return true;
	return false;
}

