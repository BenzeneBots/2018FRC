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

void AutonDriveStraight(double TargetDist, Drive *robotDrive){
	double DriveEncVal = (robotDrive->GetLeftEncoderValue() + robotDrive->GetRightEncoderValue())/2.0;
	if (INCHES_PER_TICK*DriveEncVal < TargetDist){
			robotDrive->TankDrive(AUTON_DRIVE_SPEED*LEFT_DRIVE_CORRECTION,AUTON_DRIVE_SPEED);
	}
	else{
			robotDrive->TankDrive(0.0,0.0);
	}
}

void AutonTurnRight(double TargetAngle,Drive *robotDrive){

	double CurrentAngle = abs(robotDrive->GetFusedHeading());

	if (CurrentAngle < TargetAngle){
			robotDrive->TankDrive(AUTON_TURN_SPEED*LEFT_DRIVE_CORRECTION,-1.0*AUTON_TURN_SPEED);
	}else{
			robotDrive->TankDrive(0.0,0.0);
	}
}


void AutonTurnLeft(double TargetAngle,Drive *robotDrive){
	double CurrentAngle = abs(robotDrive->GetFusedHeading());

	if (CurrentAngle < TargetAngle){
			robotDrive->TankDrive(-1.0*AUTON_TURN_SPEED*LEFT_DRIVE_CORRECTION,AUTON_TURN_SPEED);
	}else{
			robotDrive->TankDrive(0.0,0.0);
	}
}
void AutonLowerElevator(Elevator *robotElevator){

}
void AutonLiftToSwitch(Elevator *robotElevator){

}
void AutonLiftToScale(Elevator *robotElevator){

}
void AutonOuttake(Intake* robotIntake){

}
void AutonDeployIntake(Intake* robotIntake){
}




