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
#define TICKS_PER_ROTATION 4096//measured
#define INCHES_PER_TICK 0.0019175//calculated, but may need tuning
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


void Elevator::SetToOutput(double ElevatorSpeed){

		elevatorMotor->Set(ControlMode::PercentOutput,ElevatorSpeed);

}
void Elevator::SetEncoderPosition(int pos){
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
	//

	//Configure limits at 0 and 5 rotations forward
	elevatorMotor->ConfigForwardSoftLimitThreshold(5.0*TICKS_PER_ROTATION, 10);
	elevatorMotor->ConfigReverseSoftLimitThreshold(0,10);

	 //Enable Soft Limits
	 elevatorMotor->ConfigForwardSoftLimitEnable(true,10);
	 elevatorMotor->ConfigReverseSoftLimitEnable(true,10);

	 elevatorMotor->OverrideLimitSwitchesEnable(true);
}

void Elevator::SetElevatorSetPoint(double pos){
	targetHeight = pos;
}

void Elevator::MoveElevatorToSetPoint(){//TODO implement this
}


} /* namespace Elevator */
