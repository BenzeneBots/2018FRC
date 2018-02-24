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
#define RIGHT_DRIVE_CORRECTION 0.96 //TODO tune this
#define RESET_TIMEOUT 10

//Auton DriveStraight Ramping Constants
#define MIN_STRAIGHT_DRIVE_SPEED 0.15
#define MIN_STRAIGHT_DIST_DIFFERENCE 12.0
#define STEP1_STRAIGHT_DIST_DIFFERENCE 24.0
#define STEP2_STRAIGHT_DIST_DIFFERENCE 36.0
#define STRAIGHT_RAMP_RATE 0.55
#define STEP1_STRAIGHT_DRIVE_SPEED 0.30
#define STEP2_STRIAGHT_DRIVE_SPEED 0.45
#define MAX_STRAIGHT_DRIVE_SPEED 0.60

//Auton Turning Ramping Constants
#define MIN_TURN_DRIVE_SPEED 0.10
#define MIN_TURN_ANG_DIFFERENCE 15.0
#define TURN_RAMP_RATE 0.2
#define MAX_TURN_DRIVE_SPEED 0.5

//Init Auton Timers
Timer *autonTimer = new Timer();

bool AutonDriveStraight(double TargetDist, Drive *drive){
	if (drive->GetAverageEncoderDistance() < TargetDist){
		//drive->TankDrive(-1.0*AUTON_DRIVE_SPEED,-1.0*AUTON_DRIVE_SPEED*RIGHT_DRIVE_CORRECTION);

		//drive speed ramping
		//double autonStraightSpeed = drive->AutonRamping(distDifference,MIN_STRAIGHT_DIST_DIFFERENCE,MIN_STRAIGHT_DRIVE_SPEED,STRAIGHT_RAMP_RATE,MAX_STRAIGHT_DRIVE_SPEED);
		double distDifference = (TargetDist - drive->GetAverageEncoderDistance());
		printf("Dist Difference %f \n", distDifference);

		//double autonDriveSpeed = drive->AutonRamping1(distDifference,MIN_STRAIGHT_DIST_DIFFERENCE,MIN_STRAIGHT_DRIVE_SPEED,STRAIGHT_RAMP_RATE,MAX_STRAIGHT_DRIVE_SPEED);
		double autonDriveSpeed = drive->AutonRamping2(distDifference,MIN_STRAIGHT_DRIVE_SPEED, STEP1_STRAIGHT_DRIVE_SPEED,STEP2_STRIAGHT_DRIVE_SPEED,MAX_STRAIGHT_DRIVE_SPEED,MIN_STRAIGHT_DIST_DIFFERENCE,STEP1_STRAIGHT_DIST_DIFFERENCE,STEP2_STRAIGHT_DIST_DIFFERENCE);
		printf("RampSpeed %f \n", autonDriveSpeed);
		drive->TankDrive(-1.0*autonDriveSpeed,-1.0*autonDriveSpeed*RIGHT_DRIVE_CORRECTION);



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
	double angDifference = CurrentAngle - (-1.0* TargetAngle);
	if (CurrentAngle > (-1.0 * TargetAngle)){
			drive->TankDrive(AUTON_TURN_SPEED, -1.0 * AUTON_TURN_SPEED*RIGHT_DRIVE_CORRECTION);

			//turn speed ramping
			//double autonTurnSpeed = drive->AutonRamping1(angDifference,MIN_TURN_ANG_DIFFERENCE,MIN_TURN_DRIVE_SPEED,TURN_RAMP_RATE,MAX_TURN_DRIVE_SPEED);
			//drive->TankDrive(autonTurnSpeed, -1.0 * autonTurnSpeed *RIGHT_DRIVE_CORRECTION);
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
	double angDifference = CurrentAngle - (-1.0* TargetAngle);
	if (CurrentAngle < TargetAngle){
			drive->TankDrive(-1.0 * AUTON_TURN_SPEED, AUTON_TURN_SPEED*RIGHT_DRIVE_CORRECTION);

			//turn speed ramping
			//double autonTurnSpeed = drive->AutonRamping1(angDifference,MIN_TURN_ANG_DIFFERENCE,MIN_TURN_DRIVE_SPEED,TURN_RAMP_RATE,MAX_TURN_DRIVE_SPEED);
			//drive->TankDrive(autonTurnSpeed, -1.0 * autonTurnSpeed *RIGHT_DRIVE_CORRECTION);
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




