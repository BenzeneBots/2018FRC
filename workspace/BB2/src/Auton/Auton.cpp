/*
 * Auton.cpp
 *
 *  Created on: Feb 19, 2018
 *      Author: Sanket Nayak
 */
#include <Auton/Auton.h>
#include <ctre/Phoenix.h>
#include <WPILib.h>
#include <Timer.h>

#define AUTON_DRIVE_SPEED 0.35
#define AUTON_TURN_SPEED 0.20
#define LEFT_DRIVE_CORRECTION 1.0
#define RESET_TIMEOUT 10

#define ELEVATOR_BOTTOM_HEIGHT -1200
#define ELEVATOR_SWITCH_HEIGHT 3467
#define ELEVATOR_SCALE_HEIGHT 13000

//Init Auton Timers
Timer *autonTimer = new Timer();


bool AutonDriveStraight(double TargetDist, Drive *drive){
	if (drive->GetAverageEncoderDistance() < TargetDist){
			drive->TankDrive(-1.0*AUTON_DRIVE_SPEED*LEFT_DRIVE_CORRECTION,-1.0*AUTON_DRIVE_SPEED);
			return false;
	}
	else{
			drive->TankDrive(0.0,0.0);

			//Reset Values
			drive->ResetEncoders();
			drive->ResetFusedHeading();
			drive->ResetYaw();

			return true;

	}
}

bool AutonTurnRight(double TargetAngle,Drive *drive){

	double CurrentAngle = drive->GetYaw();

	if (CurrentAngle > (-1.0 * TargetAngle)){
			drive->TankDrive(AUTON_TURN_SPEED, -1.0 * AUTON_TURN_SPEED);
			return false;
	}
	else{
			drive->TankDrive(0.0,0.0);

			//Reset Values
			drive->ResetEncoders();
			drive->ResetFusedHeading();
			drive->ResetYaw();

			return true;

	}
}


bool AutonTurnLeft(double TargetAngle,Drive *drive){
	double CurrentAngle = drive->GetYaw();

	if (CurrentAngle < TargetAngle){
		drive->TankDrive(-1.0 * AUTON_TURN_SPEED, AUTON_TURN_SPEED);
			return false;
		}
	else{
			drive->TankDrive(0.0,0.0);

			//Reset Values
			drive->ResetEncoders();
			drive->ResetFusedHeading();
			drive->ResetYaw();

			return true;
	}
}
bool AutonSetHeight(double targetHeight,Elevator *elevator){
	if(elevator->SetElevatorTarget(targetHeight)){
		return true;
	}
	else{
		return false;
	}
}
bool AutonMoveToHeight(Elevator *elevator){
	if(elevator->MoveElevator(0.0)){
		return true;
	}
	else{
	return false;
	}
}

bool AutonIntake(Intake *robotIntake){
	robotIntake->IntakeCubes();
	return true;
}

bool AutonOuttake(Intake* robotIntake){
	robotIntake->OuttakeCubes();
	return true;
	printf("Intake Timer: %f\n", autonTimer->Get());
}

bool AutonStopIntake(Intake* robotIntake){
	robotIntake->StopIntake();
	return true;
}

bool AutonStowIntake(Intake* robotIntake){
	if(robotIntake->IsIntakeDeployed()){
		robotIntake->StowIntake();
		return false;
	}
	else{
		return true;
	}
}

bool AutonDeployIntake(Intake* robotIntake){
	if(robotIntake->IsIntakeDeployed()){
			return true;
		}
		else{
			robotIntake->DeployIntake();
			return false;
		}
}


bool AutonScoreSwitch(Elevator* robotElevator, Intake* robotIntake){//raises elevator to switch height, scores cube, and lowers (in that order)
	enum stateMachine {setElevator1, raiseElevator, deployIntake, outtake, stowIntake, setElevator2, lowerIntake};
	stateMachine currentState = setElevator1;

	Timer *initTimer = new Timer();

	case setElevator1:
		if(AutonSetHeight(ELEVATOR_SWITCH_HEIGHT,robotElevator)){
			stateMachine = raiseElevator;
		}
		break;
	case raiseElevator:
		if(AutonMoveToHeight(robotElevator)){
		robotElevator->SetToOutput(0.1);
		stateMachine = deployIntake;
		//Initiates timer for deploying
		initTimer->Reset();
		initTimer->Start();
		}
		break;
	case deployIntake:
			if(AutonDeployIntake(robotIntake) && (initTimer->Get() > 2.0)){
			initTimer->Stop();
			//Initiates timer for outtake
			initTimer->Reset();
			initTimer->Start();
			stateMachine = outtake;
		}
		break;
	case outtake:
		printf("Outtake Timer: %f\n", initTimer->Get());
			if(AutonOuttake(robotIntake) && (initTimer->Get() > 0.5)){
			AutonStopIntake(robotIntake);
			initTimer->Stop();
			//Initiates timer for stowage
			initTimer->Reset();
			initTimer->Start();
			stateMachine = stowIntake;
		}
		break;
	case stowIntake:
		if(AutonStowIntake(robotIntake) && (initTimer->Get() > 1.5)){
			stateMachine = setElevator2;
		}
		break;
		case setElevator2:
		if(AutonSetHeight(ELEVATOR_BOTTOM_HEIGHT,robotElevator)){
			stateMachine = lowerIntake;
		}
		break;
	case lowerIntake:
		if(AutonMoveToHeight(robotElevator)){
			return true;
		}
		break;
}




