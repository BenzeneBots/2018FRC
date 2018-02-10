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


/*void Elevator::SetToOutput(double ElevatorSpeed){

		elevatorMotor->Set(ControlMode::PercentOutput,ElevatorSpeed);

}*/
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
void Elevator::PIDInit(){
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
	elevatorMotor->Config_kF(0,0.0,10);
	elevatorMotor->Config_kP(0, 0.0,10);
	elevatorMotor->Config_kI(0, 0.0,10);
	elevatorMotor->Config_kD(0, 0.0,10);
}
void Elevator::MoveElevatorToSetPoint(bool button1,bool button2,bool button3,double ElevatorSpeed){//TODO implement this
	/* on button1 press enter closed-loop mode on target position */
			if (button1) {
				/* Position mode - button just pressed */
				double targetPositionRotations = 0.0 * TICKS_PER_ROTATION; /* 0 Rotations in either direction */
				elevatorMotor->Set(ControlMode::Position, targetPositionRotations); /* 0 rotations in either direction */
			}
			if (button2) {
				double targetPositionRotations = 5.0 * TICKS_PER_ROTATION; /* 5 Rotations in either direction */
				elevatorMotor->Set(ControlMode::Position, targetPositionRotations); /* 5 rotations in either direction */
			}
			if (button3) {
				double targetPositionRotations = 10 * TICKS_PER_ROTATION; /* 10 Rotations in either direction */
				elevatorMotor->Set(ControlMode::Position, targetPositionRotations); /* 10 rotations in either direction */
			}
			else {
				elevatorMotor->Set(ControlMode::PercentOutput,ElevatorSpeed);
			}
}


} /* namespace Elevator */
