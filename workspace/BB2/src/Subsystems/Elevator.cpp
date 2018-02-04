/*
 * Elevator.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: Murali
 */

#include <Subsystems/Elevator.h>
#include <WPILib.h>
#include <ctre/Phoenix.h>

//constants
//#define INCHES_PER_TICK 0.04125//need to actually measure
#define INCHES_PER_ROTATION 120//not measured yet
/*

 */

namespace Elevator {

Elevator::Elevator(int elevatorPort) {
	elevatorMotor = new TalonSRX(elevatorPort);
	// TODO Auto-generated constructor stub
}
void Elevator::SetToSpeed(double IntakeSpeed){
	elevatorMotor->Set(ControlMode::PercentOutput,IntakeSpeed);
}
void Elevator::ResetEncoder(){
	//Resets Encoders to 0
	elevatorMotor->GetSensorCollection().SetQuadraturePosition(0, 0); //should the timeoutMS be 0
}

double Elevator::getElevatorEncoderValue(){

	double rawEncVal = elevatorMotor->GetSensorCollection().GetQuadraturePosition();
	return rawEncVal;
	printf("Enc Val: %f\n",rawEncVal);
}

double Elevator::getElevatorRate(){
	double VelocityVal = elevatorMotor->GetSensorCollection().GetQuadratureVelocity();
	return VelocityVal;
}

void Elevator::LiftElevatorToHeight(double){

}



} /* namespace Elevator */
