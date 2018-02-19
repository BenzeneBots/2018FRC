/*
 * Auton.cpp
 *
 *  Created on: Feb 19, 2018
 *      Author: Sanket Nayak
 */
#include <Auton/Auton.h>
#include <ctre/Phoenix.h>
#include <WPILib.h>


#define AUTON_DRIVE_SPEED 0.5
#define AUTON_TURN_SPEED 0.1
#define LEFT_DRIVE_CORRECTION 1.0
#define RESET_TIMEOUT 10

bool AutonDriveStraight(double TargetDist, Drive *drive){

	if (drive->GetAverageEncoderDistance() < TargetDist){
			drive->TankDrive(AUTON_DRIVE_SPEED*LEFT_DRIVE_CORRECTION, AUTON_DRIVE_SPEED);
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

	double CurrentAngle = abs(drive->GetFusedHeading());

	if (CurrentAngle < TargetAngle){
			drive->TankDrive(AUTON_TURN_SPEED*LEFT_DRIVE_CORRECTION,-1.0*AUTON_TURN_SPEED);
			return false;
	}else{
			drive->TankDrive(0.0,0.0);

			//Reset Values
			drive->ResetEncoders();
			drive->ResetFusedHeading();
			drive->ResetYaw();

			return true;

	}
}


bool AutonTurnLeft(double TargetAngle,Drive *drive){
	double CurrentAngle = abs(drive->GetFusedHeading());

	if (CurrentAngle < TargetAngle){
			drive->TankDrive(-1.0*AUTON_TURN_SPEED*LEFT_DRIVE_CORRECTION,AUTON_TURN_SPEED);
			return false;
	}else{
			drive->TankDrive(0.0,0.0);

			//Reset Values
			drive->ResetEncoders();
			drive->ResetFusedHeading();
			drive->ResetYaw();

			return true;
	}
}

bool AutonMoveElevatorToHeight(double, Elevator *elevator){
	return true;
}

bool AutonOuttake(Intake* robotIntake){
	return true;
}

bool AutonDeployIntake(Intake* robotIntake){
	return true;
}




