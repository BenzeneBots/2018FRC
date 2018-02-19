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
#define SET_POINT_FlOOR 0
#define SET_POINT_SWITCH 36
#define SET_POINT_SCALE 72

#define ELEVATOR_SPEED 0.7
#define MAX_ELEVATOR_HEIGHT 13000 //15000
#define MIN_ELEVATOR_HEIGHT -1750 //1650

#define CONST_BACKDRIVE_PREVENTION 0.1

Elevator::Elevator(int elevatorPort) {
	elevatorMotor = new TalonSRX(elevatorPort);
	elevatorMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0); //sets the quad encoder as the primary sensor. What do PIDLoop and timeoutMS (the parameters) do?
	elevatorMotor->SetInverted(true); //positive motor = upward motion
	elevatorMotor->SetSensorPhase(true); //ensure sensor phase matches

	//Enable Soft Limits on init
	elevatorMotor->ConfigForwardSoftLimitThreshold(MAX_ELEVATOR_HEIGHT, 10);
	elevatorMotor->ConfigReverseSoftLimitThreshold(MIN_ELEVATOR_HEIGHT,10);
	elevatorMotor->ConfigForwardSoftLimitEnable(true,10);
	elevatorMotor->ConfigReverseSoftLimitEnable(true,10);

	elevatorState = joystick;
	elevatorTargetPos = 0;

	//Auto-generated constructor stub
}


void Elevator::SetToOutput(double elevatorSpeed){
	double elevatorPos = GetElevatorPosition();
	if(((elevatorPos > MAX_ELEVATOR_HEIGHT) && (elevatorSpeed < 0)) ||
			((elevatorPos < MIN_ELEVATOR_HEIGHT) && elevatorSpeed > 0) ||
			(((elevatorPos > MIN_ELEVATOR_HEIGHT)) && (elevatorPos < MAX_ELEVATOR_HEIGHT))){
		if(elevatorSpeed > 0.1){//override the other stuff only if the speed is greater that 0.1 (if it's intentional)
			elevatorMotor->Set(ControlMode::PercentOutput,elevatorSpeed);
		}
	}
}

void Elevator::SetEncoderPosition(int pos){
	elevatorMotor->GetSensorCollection().SetQuadraturePosition(pos, 0); //should the timeoutMS be 0
}

double Elevator::GetElevatorPosition(){
	double rawEncVal = elevatorMotor->GetSensorCollection().GetQuadraturePosition();
	return rawEncVal;
}

double Elevator::GetElevatorRate(){
	double VelocityVal = elevatorMotor->GetSensorCollection().GetQuadratureVelocity();
	return VelocityVal;
}


void Elevator::SetElevatorTarget(double targetPosition){//TODO implement this
	elevatorTargetPos = targetPosition;
	if(this->GetElevatorPosition() < targetPosition) elevatorState = increasing;
	else if(this->GetElevatorPosition() > targetPosition) elevatorState = decreasing;
}

void Elevator::MoveElevator(double joystickVal){
	switch (elevatorState){
	case increasing:
		if(this->GetElevatorPosition() > elevatorTargetPos) elevatorState = joystick;
		if(abs(joystickVal) > 0.1) elevatorState = joystick;
		elevatorMotor->Set(ControlMode::PercentOutput, ELEVATOR_SPEED);
		break;
	case decreasing:
		if(this->GetElevatorPosition() < (elevatorTargetPos + 1200)) elevatorState = joystick;
		if(abs(joystickVal) > 0.1) elevatorState = joystick;
		elevatorMotor->Set(ControlMode::PercentOutput, -1.0 * ELEVATOR_SPEED);
		break;
	case joystick:
		this->SetToOutput(joystickVal);
	}



}
