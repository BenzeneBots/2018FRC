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
#define TICKS_PER_ROTATION 120//not measured yet
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

void Elevator::LimitElevator(){
	//Configure to limit to 5 Rotations Forward
	 elevatorMotor->ConfigForwardSoftLimitThreshold(+5*TICKS_PER_ROTATION, 10);
	 //COnfigure to limit to 0 Rotations Backward
	 elevatorMotor->ConfigReverseSoftLimitThreshold(0,10);

	 //Enable Soft Limits
	 elevatorMotor->ConfigForwardSoftLimitEnable(true,10);
	 elevatorMotor->ConfigReverseSoftLimitEnable(true,10);
}


} /* namespace Elevator */
