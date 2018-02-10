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
#define INTAKE_SPEED 0.75

namespace Intake {

Intake::Intake(int intake1Port, int intake2Port, int anglePort) {
	intake1 = new Victor(intake1Port);
	intake2 = new Victor(intake2Port);
	angleMotor = new TalonSRX(anglePort);

}

void Intake::IntakeCubes(){
	intake1->Set(INTAKE_SPEED);
	intake2->Set(INTAKE_SPEED);
}

void Intake::OuttakeCubes(){
	intake1->Set(-1.0 * INTAKE_SPEED);
	intake2->Set(-1.0 * INTAKE_SPEED);
}

void Intake::StopIntake(){
	intake1->Set(0.0);
	intake2->Set(0.0);
}

void Intake::DeployIntake(){

}

void Intake::StowIntake(){

}
//Don't deploy intake if the intake is stowed
}
