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
#define SET_POINT_SWITCH 3467
#define SET_POINT_SCALE 72

#define ELEVATOR_RISING_SPEED .99
#define ELEVATOR_LOWERING_SPEED 0.75
#define MAX_ELEVATOR_HEIGHT 14000 //15000
#define MIN_ELEVATOR_HEIGHT -1750 //1650

#define ELEVATOR_BOTTOM_HEIGHT -600
#define ELEVATOR_SWITCH_HEIGHT 5500	//TODO untested
#define ELEVATOR_SCALE_HEIGHT 13200 //untested

#define CONST_BACKDRIVE_PREVENTION 0.1
#define FF_MAXe		2785

const static int PID_PRIMARY = 3;
const uint8_t kTimeoutMs = 10;


struct Gains {
	double kP, kI, kD, kF;
	double kIzone;
	double kPeakOutput;
};

constexpr static Gains kGains_MMe =      { 0.3, 0.002, 30.0, 1023.0/FF_MAXe,  400,  1.00 }; // measured 3200 max velocity
const uint8_t kTO = 10;

Elevator::Elevator(int elevatorPort) {
	elevatorMotor = new TalonSRX(elevatorPort);
	elevatorMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0); //sets the quad encoder as the primary sensor. What do PIDLoop and timeoutMS (the parameters) do?
	elevatorMotor->SetInverted(true); //positive motor = upward motion
	elevatorMotor->SetSensorPhase(true); //ensure sensor phase matches

	//Enable Soft Limits on init
	elevatorMotor->ConfigForwardSoftLimitThreshold(MAX_ELEVATOR_HEIGHT, 10);
	elevatorMotor->ConfigReverseSoftLimitThreshold(MIN_ELEVATOR_HEIGHT,10);
	elevatorMotor->ConfigForwardSoftLimitEnable(true,10);
	elevatorMotor->ConfigReverseSoftLimitEnable(true,10);

	elevatorMotor->ConfigOpenloopRamp(.25, 0.0);

	elevatorState = joystick;
	elevatorTargetPos = 0;


	// Elevator PID
	elevatorMotor->Config_kF( PID_PRIMARY, kGains_MMe.kF, kTO );
	elevatorMotor->Config_kP( PID_PRIMARY, kGains_MMe.kP, kTO );
	elevatorMotor->Config_kI( PID_PRIMARY, kGains_MMe.kI, kTO);
	elevatorMotor->Config_kD( PID_PRIMARY, kGains_MMe.kD, kTO);
	elevatorMotor->Config_IntegralZone( PID_PRIMARY, kGains_MMe.kIzone, kTO );
	elevatorMotor->ConfigPeakOutputForward(1.0, kTO );
	elevatorMotor->ConfigPeakOutputReverse(-1.0, kTO);

	double nuSp = 1.5 * 5.0 * 260.9;	// 3ft/s
	elevatorMotor->ConfigMotionAcceleration( nuSp * 1.5, 0 );
	elevatorMotor->ConfigMotionCruiseVelocity( nuSp, 0 );

	elevatorMotor->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);
	lockElevator = true;
}


void Elevator::SetToOutput(double elevatorSpeed){
	double elevatorPos = GetElevatorPosition();
	if(((elevatorPos > MAX_ELEVATOR_HEIGHT) && (elevatorSpeed < 0)) ||
			((elevatorPos < MIN_ELEVATOR_HEIGHT) && elevatorSpeed > 0) ||
			(((elevatorPos > MIN_ELEVATOR_HEIGHT)) && (elevatorPos < MAX_ELEVATOR_HEIGHT))){

		if(elevatorSpeed >= 0){
			elevatorMotor->Set(ControlMode::PercentOutput,elevatorSpeed);

		}
		else{
			elevatorMotor->Set(ControlMode::PercentOutput,0.6 * elevatorSpeed);
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

double Elevator::GetElevatorHeight(){
	double elevatorHeight = this->GetElevatorPosition()*INCHES_PER_TICK;
	return elevatorHeight;
}

double Elevator::GetElevatorRate(){
	double VelocityVal = elevatorMotor->GetSensorCollection().GetQuadratureVelocity();
	return VelocityVal;
}


bool Elevator::SetElevatorTarget(double targetPosition){//TODO implement this
	elevatorTargetPos = targetPosition;
	if(this->GetElevatorPosition() < targetPosition) elevatorState = increasing;
	else if(this->GetElevatorPosition() > targetPosition) elevatorState = decreasing;
	return true;
}

bool Elevator::MoveElevator(double joystickVal){
	switch (elevatorState){
	case increasing:
		if(this->GetElevatorPosition() > elevatorTargetPos){
			elevatorState = joystick;
			this->SetToOutput(CONST_BACKDRIVE_PREVENTION);
			return true;
		}
		if(abs(joystickVal) > 0.1) elevatorState = joystick;
		elevatorMotor->Set(ControlMode::PercentOutput, ELEVATOR_RISING_SPEED);
		break;
	case decreasing:
		printf("Case decreasing");
		if(this->GetElevatorPosition() < (elevatorTargetPos + 1200)){
			elevatorState = joystick;
			this->SetToOutput(CONST_BACKDRIVE_PREVENTION);
			return true;
		}
		if(abs(joystickVal) > 0.1) elevatorState = joystick;
		elevatorMotor->Set(ControlMode::PercentOutput, -1.0 * ELEVATOR_LOWERING_SPEED);
		break;
	case joystick:
		this->SetToOutput(joystickVal + CONST_BACKDRIVE_PREVENTION);
		break;
	default:
		break;
	}

	return false;

}

void Elevator::SetJoystickControl(){
	elevatorState = joystick;
}

void Elevator::MagicElevator(bool floorHeight,bool switchHeight,bool scaleHeight,double joyAxis){

	if(fabs(joyAxis)>=0.3){ //if the Joystick Axis is outside of the deadband, set the elevator output
		this->SetToOutput(joyAxis + CONST_BACKDRIVE_PREVENTION);
		lockElevator = true; //Make sure the elevator doesn't fall after this (place hold current)
	}else{
		if(floorHeight){ //If the button for the bottom height is pressed, move to height
			elevatorMotor->Set(ControlMode::MotionMagic,ELEVATOR_BOTTOM_HEIGHT);
			lockElevator = false; //Leave the Elevator in Motion Magic Mode
		}
		else if(switchHeight){
			elevatorMotor->Set(ControlMode::MotionMagic,ELEVATOR_SWITCH_HEIGHT);
			lockElevator = false;
		}
		else if(scaleHeight){
			elevatorMotor->Set(ControlMode::MotionMagic,ELEVATOR_SCALE_HEIGHT);
			lockElevator = false;
		}else{
			if(lockElevator){  //if joystick is inside deadband and the elevator is not in Motion Magic Mode, prevent backdrive
				this->SetToOutput(CONST_BACKDRIVE_PREVENTION);
			}
		}
	}
}

void Elevator::BenzeneElevator(bool floorHeight,bool switchHeight,bool scaleHeight,double joyAxis){
	double addToTarget = joyAxis*7000;
	double joyTarget = elevatorMotor->GetSensorCollection().GetQuadraturePosition() + addToTarget;

	if(fabs(joyAxis)>=0.5){ //if the Joystick Axis is outside of the deadband, set the elevator output
		elevatorMotor->SetIntegralAccumulator(0.0,0,0);
		elevatorMotor->Set(ControlMode::MotionMagic,joyTarget);
	}else{
		if(floorHeight){ //If the button for the bottom height is pressed, move to height
			elevatorMotor->SetIntegralAccumulator(0.0,0,0);
			elevatorMotor->Set(ControlMode::MotionMagic,ELEVATOR_BOTTOM_HEIGHT);
		}
		else if(switchHeight){
			elevatorMotor->SetIntegralAccumulator(0.0,0,0);
			elevatorMotor->Set(ControlMode::MotionMagic,ELEVATOR_SWITCH_HEIGHT);
		}
		else if(scaleHeight){
			elevatorMotor->SetIntegralAccumulator(0.0,0,0);
			elevatorMotor->Set(ControlMode::MotionMagic,ELEVATOR_SCALE_HEIGHT);
		}
		else{
			if(fabs(joyAxis)>=0.1){ //if the Joystick Axis is outside of the deadband, set the elevator output
				elevatorMotor->SetIntegralAccumulator(0.0,0,0);
				elevatorMotor->Set(ControlMode::MotionMagic,joyTarget);
			}
		}
	}
}
