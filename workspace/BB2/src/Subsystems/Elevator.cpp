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
#define TICKS_PER_ROTATION 4096//not measured yet
#define INCHES_PER_TICK .01416//not measured yet
#define ELEVATOR_SPEED 0.7
/*

 */

namespace Elevator {

Elevator::Elevator(int elevatorPort, int topSwitchPort, int bottomSwitchPort) {
	elevatorMotor = new TalonSRX(elevatorPort);
	bottomSwitch = new DigitalInput(bottomSwitchPort);
	topSwitch = new DigitalInput(topSwitchPort);
	targetHeight = 0;
	//Auto-generated constructor stub
}


void Elevator::SetToSpeed(double IntakeSpeed){

		elevatorMotor->Set(ControlMode::PercentOutput,IntakeSpeed);

}
void Elevator::SetEncoderPosition(int pos){
	//Resets Encoders to 0
	elevatorMotor->GetSensorCollection().SetQuadraturePosition(pos, 0); //should the timeoutMS be 0
}

double Elevator::GetElevatorPosition(){

	double rawEncVal = elevatorMotor->GetSensorCollection().GetQuadraturePosition();
	return rawEncVal;
	printf("Enc Val: %f\n",rawEncVal);
}

double Elevator::GetElevatorRate(){
	double VelocityVal = elevatorMotor->GetSensorCollection().GetQuadratureVelocity();
	return VelocityVal;
}

void Elevator::EnableSoftLimits(){
	//Configure to limit to 5 Rotations Forward
	//elevatorMotor->ConfigForwardSoftLimitThreshold(5.0*TICKS_PER_ROTATION, 10);
	 elevatorMotor->ConfigForwardSoftLimitThreshold(1.0, 10);
	 //Configure to limit to 5 Rotations Backward
	 //elevatorMotor->ConfigReverseSoftLimitThreshold(0,10);
	 elevatorMotor->ConfigReverseSoftLimitThreshold(-1.0,10);

	 //Enable Soft Limits
	 elevatorMotor->ConfigForwardSoftLimitEnable(false,10);
	 elevatorMotor->ConfigReverseSoftLimitEnable(true,10);

	 elevatorMotor->OverrideLimitSwitchesEnable(true);
}

void Elevator::SetElevatorSetPoint(double pos){
	targetHeight = pos;
}

void Elevator::MoveElevatorToSetPoint(){//TODO implement this
}


} /* namespace Elevator */
