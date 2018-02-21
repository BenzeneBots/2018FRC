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
#define RIGHT_DRIVE_CORRECTION 0.96
#define RESET_TIMEOUT 10

//Init Auton Timers
Timer *autonTimer = new Timer();


bool AutonDriveStraight(double TargetDist, Drive *drive){
	if (drive->GetAverageEncoderDistance() < TargetDist){
			drive->TankDrive(-1.0*AUTON_DRIVE_SPEED,-1.0*AUTON_DRIVE_SPEED*RIGHT_DRIVE_CORRECTION);
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
		drive->TankDrive(-1.0 * AUTON_TURN_SPEED, AUTON_TURN_SPEED*RIGHT_DRIVE_CORRECTION);
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




