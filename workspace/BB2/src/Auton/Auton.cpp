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
#define INCHES_PER_TICK 0.00460194335
#define RESET_TIMEOUT 10

bool AutonDriveStraight(double TargetDist, Drive *drive){
	double DriveEncVal = (drive->GetLeftEncoderValue() + drive->GetRightEncoderValue())/2.0;
	if (INCHES_PER_TICK*DriveEncVal < TargetDist){
			drive->TankDrive(AUTON_DRIVE_SPEED*LEFT_DRIVE_CORRECTION, AUTON_DRIVE_SPEED);
			return false;
	}
	else{
			drive->TankDrive(0.0,0.0);
			return true;
	}
}

void AutonTurnRight(double TargetAngle,Drive *drive){

	double CurrentAngle = abs(drive->GetFusedHeading());

	if (CurrentAngle < TargetAngle){
			drive->TankDrive(AUTON_TURN_SPEED*LEFT_DRIVE_CORRECTION,-1.0*AUTON_TURN_SPEED);
	}else{
			drive->TankDrive(0.0,0.0);
	}
}


void AutonTurnLeft(double TargetAngle,Drive *drive){
	double CurrentAngle = abs(drive->GetFusedHeading());

	if (CurrentAngle < TargetAngle){
			drive->TankDrive(-1.0*AUTON_TURN_SPEED*LEFT_DRIVE_CORRECTION,AUTON_TURN_SPEED);
	}else{
			drive->TankDrive(0.0,0.0);
	}
}

void AutonMoveElevatorToHeight(double, Elevator *elevator){

}

void AutonOuttake(Intake* robotIntake){

}

void AutonDeployIntake(Intake* robotIntake){
}




