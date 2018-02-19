/*
 * Auton.cpp
 *
 *  Created on: Feb 18, 2018
 *      Author: Sanket Nayak
 */

#include <Auton/AutonTasks.h>
#include <ctre/Phoenix.h>
#include <WPILib.h>
#include <Subsystems/Drive.h>

#define AUTON_DRIVE_SPEED 0.5
#define AUTON_TURN_SPEED 0.1
#define LEFT_DRIVE_CORRECTION 1.0
#define INCHES_PER_TICK 0.00460194335
#define RESET_TIMEOUT 10

Auton::Auton(int port) {
	// TODO Auto-generated constructor stub
	_pidgey = new PigeonIMU(port);
}

bool Auton::AutonDriveStraight(double TargetDist, Drive *robotDrive){
	double DriveEncVal = (robotDrive->GetLeftEncoderValue() + robotDrive->GetRightEncoderValue())/2.0;
	if (INCHES_PER_TICK*DriveEncVal < TargetDist){
			robotDrive->TankDrive(AUTON_DRIVE_SPEED*LEFT_DRIVE_CORRECTION,AUTON_DRIVE_SPEED);
			return false;
	}else{
			robotDrive->TankDrive(0.0,0.0);
			return true;
	}
}

bool Auton::AutonTurnRight(double TargetAngle,Drive *robotDrive){
	double ypr_array[3];
	_pidgey->GetYawPitchRoll(ypr_array);
	double CurrentAngle = abs(ypr_array[0]);


	if (CurrentAngle < TargetAngle){
			robotDrive->TankDrive(AUTON_TURN_SPEED*LEFT_DRIVE_CORRECTION,-1.0*AUTON_TURN_SPEED);
			return false;
	}else{
			robotDrive->TankDrive(0.0,0.0);
			return true;
	}
}

bool Auton::Reset(Drive *robotDrive){
	_pidgey->SetYaw(0.0,10);
	robotDrive->ResetEncoders();
	return true;
}

bool Auton::AutonTurnLeft(double TargetAngle,Drive *robotDrive){
	double ypr_array[3];
	_pidgey->GetYawPitchRoll(ypr_array);
	double CurrentAngle = abs(ypr_array[0]);


	if (CurrentAngle < TargetAngle){
			robotDrive->TankDrive(-1.0*AUTON_TURN_SPEED*LEFT_DRIVE_CORRECTION,AUTON_TURN_SPEED);
			return false;
	}else{
			robotDrive->TankDrive(0.0,0.0);
			return true;
	}
}
bool Auton::AutonLowerElevator(Elevator *robotElevator){
	return true;
}
bool Auton::AutonLiftToSwitch(Elevator *robotElevator){
	return true;
}
bool Auton::AutonLiftToScale(Elevator *robotElevator){
	return true;
}
bool Auton::AutonOuttake(Intake* robotIntake){
	return true;
}
bool Auton::AutonDeployIntake(Intake* robotIntake){
	return true;
}


