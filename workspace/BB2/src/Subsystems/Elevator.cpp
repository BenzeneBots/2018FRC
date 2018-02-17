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
#define SET_POINT_FlOOR 0
#define SET_POINT_SWITCH 36
#define SET_POINT_SCALE 72

#define MAX_ELEVATOR_HEIGHT 16500 //16000
#define MIN_ELEVATOR_HEIGHT -1750 //1650

Elevator::Elevator(int elevatorPort) {
	elevatorMotor = new TalonSRX(elevatorPort);
	elevatorMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0); //sets the quad encoder as the primary sensor. What do PIDLoop and timeoutMS (the parameters) do?
	//Enable Soft Limits
	elevatorMotor->ConfigForwardSoftLimitThreshold(MAX_ELEVATOR_HEIGHT, 10);
	elevatorMotor->ConfigReverseSoftLimitThreshold(MIN_ELEVATOR_HEIGHT,10);
	elevatorMotor->ConfigForwardSoftLimitEnable(true,10);
	elevatorMotor->ConfigReverseSoftLimitEnable(true,10);


	targetHeight = 0;
	//Auto-generated constructor stub
}


void Elevator::SetToOutput(double elevatorSpeed){
	elevatorMotor->Set(ControlMode::PercentOutput,elevatorSpeed);
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

void Elevator::EnableSoftLimits(){
	//Configure limits at 0 and 5 rotations forward
	elevatorMotor->ConfigForwardSoftLimitThreshold(0, 10/*5.0*TICKS_PER_ROTATION, 10*/);
	elevatorMotor->ConfigReverseSoftLimitThreshold(-7000,10);

	 //Enable Soft Limits
	 elevatorMotor->ConfigForwardSoftLimitEnable(true,10);
	 elevatorMotor->ConfigReverseSoftLimitEnable(true,10);

	 //elevatorMotor->OverrideLimitSwitchesEnable(true);
}

void Elevator::SetElevatorSetPoint(double pos){
	targetHeight = pos;
}

void Elevator::PIDInit(double FVal, double PVal, double IVal, double DVal){
/* lets grab the 360 degree position of the MagEncoder's absolute position */
	int absolutePosition = elevatorMotor->GetSelectedSensorPosition(0) & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
	/* use the low level API to set the quad encoder signal */
	elevatorMotor->SetSelectedSensorPosition(absolutePosition, 0,10);

	/* choose the sensor and sensor direction */
	elevatorMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0,10);
	elevatorMotor->SetSensorPhase(true);

	/* set the peak and nominal outputs, 12V means full */
	elevatorMotor->ConfigNominalOutputForward(0,10);
	elevatorMotor->ConfigNominalOutputReverse(0,10);
	elevatorMotor->ConfigPeakOutputForward(1,10);
	elevatorMotor->ConfigPeakOutputReverse(-1,10);

	/* set closed loop gains in slot0 */
	elevatorMotor->Config_kF(0,FVal,10);
	elevatorMotor->Config_kP(0, PVal,10);
	elevatorMotor->Config_kI(0, IVal,10);
	elevatorMotor->Config_kD(0, DVal,10);
}

void Elevator::MoveElevatorToSetPoint(double targetPositionRotation){//TODO implement this
	/* on button1 press enter closed-loop mode on target position */
	elevatorMotor->Set(ControlMode::Position, targetPositionRotation);
}
