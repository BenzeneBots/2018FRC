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
#define INTAKE_SPEED 1.0
#define OUTTAKE_SPEED 0.80
#define MIDDLE_TIMEOUT 1.0


Intake::Intake(int intake1Port, int intake2Port, int clawPort, int anglePort1, int anglePort2) {
	intake1 = new Victor(intake1Port);
	intake2 = new Victor(intake2Port);
	intake1->SetInverted(true);
	intake2->SetInverted(false);
	clawActuator = new Solenoid(clawPort);
	angleActuator = new DoubleSolenoid(anglePort1, anglePort2);
	deployTimer = new Timer();
	clawTimer = new Timer();

	//Sets initial positions
	if(angleActuator->Get() == frc::DoubleSolenoid::Value::kReverse) intakeDeployedStatus = true;
	else intakeDeployedStatus = false;
	clawOpenStatus = false;

	angleActuator->Set(DoubleSolenoid::kOff);
	deployStatus = true;
	midStatus = 0;
}

void Intake::IntakeCubes(){
	intake1->Set(INTAKE_SPEED);
	intake2->Set(INTAKE_SPEED);
}

void Intake::OuttakeCubes(){
		intake1->Set(-1.0 * OUTTAKE_SPEED);
		intake2->Set(-1.0 * OUTTAKE_SPEED);
}

void Intake::StopIntake(){
	intake1->Set(0.0);
	intake2->Set(0.0);
}

void Intake::HoldIntake(){
	intake1->Set(0.13);
	intake2->Set(0.13);
}

void Intake::OpenClaw(){
	if(intakeDeployedStatus){//only open claw if intake is deployed
		clawActuator->Set(true);
		clawOpenStatus = true;
		printf("Opening Claw \n");
	}
}

void Intake::CloseClaw(){
	clawActuator->Set(false);//since extended cylinder is closed claw
	clawOpenStatus = false;
	printf("Closing Claw \n");
}

void Intake::DeployIntake(){
	angleActuator->Set(frc::DoubleSolenoid::Value::kReverse);
	intakeDeployedStatus = true;
}


void Intake::StowIntake(){//
	if(clawOpenStatus){//if claw is open, close it before stowing
		this->CloseClaw();
	}
	//now that we are sure intake is closed, proceed with stowing
	angleActuator->Set(frc::DoubleSolenoid::Value::kForward);
	intakeDeployedStatus = false;
}

bool Intake::IsIntakeDeployed(){
	return intakeDeployedStatus;
}

bool Intake::IsClawOpen(){
	return clawOpenStatus;
}

void Intake::BenzeneIntake(double joyPOV){
	// Push POV up to raise claw.
	//angleActuator->Set( DoubleSolenoid::kReverse );
	printf("MidStatus %f \n", midStatus);
	printf("POV %f \n", joyPOV);
	printf("IntakeDeployedStatus %f \n", intakeDeployedStatus);
	if( joyPOV == 180 ) { //Claw is stowed
		midStatus = 0;
		clawTimer->Reset();
		clawTimer->Start();
		intakeDeployedStatus = false;
	}
	else if(joyPOV == 90){ //Claw is Middle Position 1
		midStatus = 1;
		clawTimer->Reset();
		clawTimer->Start();
	}
	else if(joyPOV == 270){ //Claw is Middle Position 1
		midStatus = 1;
		clawTimer->Reset();
		clawTimer->Start();
	}
	else if(joyPOV == 271){ //Claw is Middle Position 2
		midStatus = 2;
		clawTimer->Reset();
		clawTimer->Start();
	}
	else if( joyPOV == 0 ) { //Claw is deployed
		midStatus = 0;
		clawTimer->Reset();
		clawTimer->Start();
		intakeDeployedStatus = true;
	}

	if(intakeDeployedStatus){
		if(midStatus == 0){
			this->DeployIntake();
		}
		else if(midStatus == 1){ //TODO Tune this
			if(clawTimer->Get() <= 1.1){
				angleActuator->Set( DoubleSolenoid::kForward );
			}else{
				angleActuator->Set(DoubleSolenoid::kOff);
			}
		}
		else if(midStatus == 2){ //TODO Tune this
			if(clawTimer->Get() <= 0.8){
				angleActuator->Set( DoubleSolenoid::kForward );
			}else{
				angleActuator->Set(DoubleSolenoid::kOff);
			}
		}
	}
	else{
		if(midStatus == 0){
			this->StowIntake();
		}
		else if(midStatus == 1){ //TODO Tune this
			if(clawTimer->Get() <= 0.285){
				angleActuator->Set( DoubleSolenoid::kReverse);
			}else{
				angleActuator->Set(DoubleSolenoid::kOff);
			}
		}
		else if(midStatus == 2){ //TODO Tune this
			if(clawTimer->Get() <= 0.2){
				angleActuator->Set( DoubleSolenoid::kReverse);
			}else{
				angleActuator->Set(DoubleSolenoid::kOff);
			}
		}
	}
}
void Intake::StopSolenoid(){
	if(intakeDeployedStatus){
			if(midStatus == 0){
				this->DeployIntake();
			}
			else if(midStatus == 1){ //TODO Tune this
				if(clawTimer->Get() <= 0.75){
					angleActuator->Set( DoubleSolenoid::kForward );
				}else{
					angleActuator->Set(DoubleSolenoid::kOff);
				}
			}
			else if(midStatus == 2){ //TODO Tune this
				if(clawTimer->Get() <= 0.8){
					angleActuator->Set( DoubleSolenoid::kForward );
				}else{
					angleActuator->Set(DoubleSolenoid::kOff);
				}
			}
		}
		else{
			if(midStatus == 0){
				this->StowIntake();
			}
			else if(midStatus == 1){ //TODO Tune this
				if(clawTimer->Get() <= 0.285){
					angleActuator->Set( DoubleSolenoid::kReverse);
				}else{
					angleActuator->Set(DoubleSolenoid::kOff);
				}
			}
			else if(midStatus == 2){ //TODO Tune this
				if(clawTimer->Get() <= 0.2){
					angleActuator->Set( DoubleSolenoid::kReverse);
				}else{
					angleActuator->Set(DoubleSolenoid::kOff);
				}
			}
		}
}
