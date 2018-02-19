/*
 * Intake.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: Murali
 */

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Talon.h>
#include <Subsystems/Intake.h>
#define INTAKE_SPEED 0.6


Intake::Intake(int intake1Port, int intake2Port, int clawPort, int anglePort1, int anglePort2) {
	intake1 = new Victor(intake1Port);
	intake2 = new Victor(intake2Port);
	clawActuator = new Solenoid(clawPort);
	angleActuator = new DoubleSolenoid(anglePort1, anglePort2);

	//defaults position to starting config
	intakeDeployedStatus = false;
	clawOpenStatus = false;

}

void Intake::IntakeCubes(){
	intake1->Set(INTAKE_SPEED);
	intake2->Set(INTAKE_SPEED);
}

void Intake::OuttakeCubes(){
	if(intakeDeployedStatus){//only outtake if intake is deployed
		intake1->Set(-1.0 * INTAKE_SPEED);
		intake2->Set(-1.0 * INTAKE_SPEED);
	}

}

void Intake::StopIntake(){
	intake1->Set(0.0);
	intake2->Set(0.0);
}

void Intake::OpenClaw(){
	//if(intakeDeployedStatus){//only open claw if intake is deployed
		clawActuator->Set(true);
		clawOpenStatus = true;
		printf("Opening Claw \n");
	//}
}

void Intake::CloseClaw(){
	clawActuator->Set(false);//since extended cylinder is closed claw
	clawOpenStatus = false;
	printf("Closing Claw \n");
}

void Intake::DeployIntake(){
	angleActuator->Set(frc::DoubleSolenoid::Value::kForward);
	intakeDeployedStatus = true;
}

void Intake::StowIntake(){//
	if(clawOpenStatus){//if claw is open, close it before stowing
		this->CloseClaw();
	}
	//now that we are sure intake is closed, proceed with stowing
	angleActuator->Set(frc::DoubleSolenoid::Value::kReverse);
	intakeDeployedStatus = false;
}

bool Intake::IsIntakeDeployed(){
	return intakeDeployedStatus;
}

bool Intake::IsClawOpen(){
	return clawOpenStatus;
}

void Intake::SetIntakeStatus(bool newStatus){
	intakeDeployedStatus = newStatus;

}

void Intake::SetClawStatus(bool newStatus){
	clawOpenStatus = newStatus;
}

