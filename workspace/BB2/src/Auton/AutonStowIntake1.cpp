/*
 * AutonStowIntake1.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#include <Auton/AutonStowIntake1.h>
#include <WPILib.h>

#define INTAKE_STOW_TIME 2.0 //retune if needed


AutonStowIntake1::AutonStowIntake1(Intake *robotIntake,double intakeAngle) {
	intake = robotIntake;
	autonTimer = new Timer();
	deployPosition = intakeAngle;
}

AutonStowIntake1::~AutonStowIntake1() {
	// TODO Auto-generated destructor stub
}

void AutonStowIntake1::Initialize(){
	autonTimer->Reset();
	autonTimer->Start();
	intake->BenzeneIntake(deployPosition);
}

bool AutonStowIntake1::Run(){
	printf("Deploying \n");
	intake->StopSolenoid();
	if(autonTimer->Get() >= 1.5){
			autonTimer->Stop();
			intake->StopSolenoid();
			return true;
		}
		return false;

}

