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

#define AUTON_DRIVE_SPEED 0.25
#define AUTON_TURN_SPEED 0.05
#define LEFT_DRIVE_CORRECTION 1.0
#define RESET_TIMEOUT 10

//Init Auton Timers
Timer *autonTimer;


bool AutonDriveStraight(double TargetDist, Drive *drive){
	printf("ShouldAutonRun %d \n", drive->GetAverageEncoderDistance() < TargetDist);
	printf("DistanceTravelled %f \n", drive->GetAverageEncoderDistance());
	if (drive->GetAverageEncoderDistance() < TargetDist){
			drive->TankDrive(-1.0*AUTON_DRIVE_SPEED*LEFT_DRIVE_CORRECTION,-1.0*AUTON_DRIVE_SPEED);
			return false;
	}
	else{
			drive->TankDrive(0.0,0.0);

			//Reset Values
			printf("finished \n");
			drive->ResetEncoders();
			drive->ResetFusedHeading();
			drive->ResetYaw();

			return true;

	}
}

bool AutonTurnRight(double TargetAngle,Drive *drive){

	double CurrentAngle = abs(drive->GetFusedHeading());

	if (CurrentAngle < TargetAngle){
			drive->ArcadeDrive(0,AUTON_TURN_SPEED);
			return false;
	}
	else{
			drive->ArcadeDrive(0.0,0.0);

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
			drive->ArcadeDrive(0,AUTON_TURN_SPEED);
			return false;
		}
	else{
			drive->ArcadeDrive(0.0,0.0);

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

bool AutonIntake(double time,Intake *robotIntake){
	autonTimer->Reset();
	autonTimer->Start();
	if(autonTimer->Get() <= time){
		robotIntake->IntakeCubes();
		return false;
	}
	else{
		autonTimer->Stop();
		return true;
	}
}

bool AutonOuttake(double time,Intake* robotIntake){
	autonTimer->Reset();
	autonTimer->Start();
	if(autonTimer->Get() <= time){
		robotIntake->OuttakeCubes();
		return false;
	}
	else{
		autonTimer->Stop();
		return true;
	}
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




