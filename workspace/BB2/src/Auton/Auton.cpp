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
#define RIGHT_DRIVE_CORRECTION 1.0 //TODO tune this
#define RESET_TIMEOUT 10

#define STRAIGHT_P_CONSTANT 0.05 //TODO tune this

//Auton DriveStraight Ramping Constants
#define MIN_STRAIGHT_DRIVE_SPEED 0.15
#define MIN_STRAIGHT_DIST_DIFFERENCE 12.0
#define STEP1_STRAIGHT_DIST_DIFFERENCE 24.0
#define STEP2_STRAIGHT_DIST_DIFFERENCE 36.0
//#define STRAIGHT_RAMP_RATE 0.55
#define STEP1_STRAIGHT_DRIVE_SPEED 0.30
#define STEP2_STRAIGHT_DRIVE_SPEED 0.45
#define MAX_STRAIGHT_DRIVE_SPEED 0.70

//Auton Left Turning Ramping Constants
#define MIN_LEFT_TURN_DRIVE_SPEED 0.06
#define MIN_LEFT_TURN_ANG_DIFFERENCE 16.0
#define STEP1_LEFT_TURN_ANG_DIFFERENCE 32.0
#define STEP2_LEFT_TURN_ANG_DIFFERENCE 48.0
//#define LEFT_TURN_RAMP_RATE 0.55
#define STEP1_LEFT_TURN_DRIVE_SPEED 0.15
#define STEP2_LEFT_TURN_DRIVE_SPEED 0.25
#define MAX_LEFT_TURN_DRIVE_SPEED 0.35

//TODO tune Right turn
//Auton Right Turning Ramping Constants
#define MIN_RIGHT_TURN_DRIVE_SPEED 0.06
#define MIN_RIGHT_TURN_ANG_DIFFERENCE 16.0
#define STEP1_RIGHT_TURN_ANG_DIFFERENCE 32.0
#define STEP2_RIGHT_TURN_ANG_DIFFERENCE 48.0
//#define RIGHT_TURN_RAMP_RATE 0.55
#define STEP1_RIGHT_TURN_DRIVE_SPEED 0.15
#define STEP2_RIGHT_TURN_DRIVE_SPEED 0.25
#define MAX_RIGHT_TURN_DRIVE_SPEED 0.35

//Init Auton Timers
Timer *autonTimer = new Timer();

bool AutonDriveStraight(double TargetDist, Drive *drive, double startYaw){
	if (drive->GetAverageEncoderDistance() < TargetDist){

		//finds difference b/w target and actual distance
		double distDifference = (TargetDist - drive->GetAverageEncoderDistance());
		//double autonDriveSpeed = drive->AutonRamping1(distDifference,MIN_STRAIGHT_DIST_DIFFERENCE,MIN_STRAIGHT_DRIVE_SPEED,STRAIGHT_RAMP_RATE,MAX_STRAIGHT_DRIVE_SPEED);

		//based on that, determines what the drive speed should be so it ramps properly
		double adjDriveSpeed= drive->AutonRamping2(distDifference,MIN_STRAIGHT_DRIVE_SPEED, STEP1_STRAIGHT_DRIVE_SPEED,STEP2_STRAIGHT_DRIVE_SPEED,MAX_STRAIGHT_DRIVE_SPEED,MIN_STRAIGHT_DIST_DIFFERENCE,STEP1_STRAIGHT_DIST_DIFFERENCE,STEP2_STRAIGHT_DIST_DIFFERENCE);
		double ratio = adjDriveSpeed/MAX_STRAIGHT_DRIVE_SPEED;

		//uses gyro to correct
		double leftVal = adjDriveSpeed;
		double rightVal = adjDriveSpeed * RIGHT_DRIVE_CORRECTION;

		double yawError = startYaw+drive->GetYaw(); //since it is zeroed before this step starts, the angle IS the error

		//multiplies by ratio
		leftVal -= ratio * (yawError * STRAIGHT_P_CONSTANT);
		rightVal += ratio * (yawError * STRAIGHT_P_CONSTANT);

		drive->TankDrive(-1.0*leftVal,-1.0*rightVal);

		return false;
	}
	else{

			drive->TankDrive(0.0,0.0);

			return true;
	}
}

bool AutonTurnRight(double TargetAngle,Drive *drive,double startYaw){
	double CurrentAngle = drive->GetYaw();
	double angDifference = CurrentAngle - (-1.0* TargetAngle);
	if (CurrentAngle > ((-1.0 * TargetAngle)+startYaw)){
			//drive->TankDrive(AUTON_TURN_SPEED, -1.0 * AUTON_TURN_SPEED);
			printf("Current Angle %f \n", CurrentAngle);

			double adjTurnSpeed= drive->AutonRamping2(angDifference,MIN_RIGHT_TURN_DRIVE_SPEED, STEP1_RIGHT_TURN_DRIVE_SPEED,STEP2_RIGHT_TURN_DRIVE_SPEED,MAX_RIGHT_TURN_DRIVE_SPEED,MIN_RIGHT_TURN_ANG_DIFFERENCE,STEP1_RIGHT_TURN_ANG_DIFFERENCE,STEP2_RIGHT_TURN_ANG_DIFFERENCE);

			//turn speed ramping
			//double autonTurnSpeed = drive->AutonRamping1(angDifference,MIN_TURN_ANG_DIFFERENCE,MIN_TURN_DRIVE_SPEED,TURN_RAMP_RATE,MAX_TURN_DRIVE_SPEED);
			drive->TankDrive(adjTurnSpeed, -1.0 * adjTurnSpeed *RIGHT_DRIVE_CORRECTION);
			return false;
	}
	else{
			drive->TankDrive(0.0,0.0);

			//Reset Values
			drive->ResetEncoders();
			drive->ResetFusedHeading();
			drive->ResetYaw();

			printf("Passing to next step \n");

			return true;

	}

}


bool AutonTurnLeft(double TargetAngle,Drive *drive,double startYaw){
	double CurrentAngle = drive->GetYaw();
	double angDifference = CurrentAngle - (-1.0* TargetAngle);
	if (CurrentAngle < (TargetAngle+startYaw)){

			//drive->TankDrive(-1.0 * AUTON_TURN_SPEED, AUTON_TURN_SPEED);
			double adjTurnSpeed= drive->AutonRamping2(angDifference,MIN_LEFT_TURN_DRIVE_SPEED, STEP1_LEFT_TURN_DRIVE_SPEED,STEP2_LEFT_TURN_DRIVE_SPEED,MAX_LEFT_TURN_DRIVE_SPEED,MIN_LEFT_TURN_ANG_DIFFERENCE,STEP1_LEFT_TURN_ANG_DIFFERENCE,STEP2_LEFT_TURN_ANG_DIFFERENCE);

			//turn speed ramping
			//double autonTurnSpeed = drive->AutonRamping1(angDifference,MIN_TURN_ANG_DIFFERENCE,MIN_TURN_DRIVE_SPEED,TURN_RAMP_RATE,MAX_TURN_DRIVE_SPEED);
			drive->TankDrive(-1.0*adjTurnSpeed,adjTurnSpeed *RIGHT_DRIVE_CORRECTION);
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




