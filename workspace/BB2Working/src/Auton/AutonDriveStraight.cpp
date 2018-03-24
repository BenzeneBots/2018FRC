/*
 * AutonDriveStraight.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#include <Auton/AutonDriveStraight.h>

//General drive constants
#define AUTON_DRIVE_SPEED 0.35
#define AUTON_TURN_SPEED 0.20
#define RIGHT_DRIVE_CORRECTION 1.0 //TODO tune this
#define RESET_TIMEOUT 10
#define STRAIGHT_P_CONSTANT 0.005 //TODO tune this


//Auton DriveStraight Ramping Constants
#define MIN_STRAIGHT_DRIVE_SPEED 0.23
#define MIN_STRAIGHT_DIST_DIFFERENCE 12.0
#define STEP1_STRAIGHT_DIST_DIFFERENCE 24.0
#define STEP2_STRAIGHT_DIST_DIFFERENCE 50.0
//#define STRAIGHT_RAMP_RATE 0.55
#define STEP1_STRAIGHT_DRIVE_SPEED 0.30
#define STEP2_STRAIGHT_DRIVE_SPEED 0.40
#define MAX_STRAIGHT_DRIVE_SPEED 0.50

#define MAX_STALL_TIME 0.25

AutonDriveStraight::AutonDriveStraight(Drive *robotDrive, double dist) {
	drive = robotDrive;
	distance = dist;
	startYaw = 0;
	frictionTimer = new Timer();
}

AutonDriveStraight::~AutonDriveStraight() {
	// TODO Auto-generated destructor stub
}

void AutonDriveStraight::Initialize(){
	drive->ResetEncoders();
	startYaw = drive->GetYaw();
	frictionTimer->Start();
	frictionTimer->Stop();
	frictionTimer->Reset();
}

bool AutonDriveStraight::Run(){
	if (drive->GetAverageEncoderDistance() >= distance){
		drive->TankDrive(0.0,0.0);
		drive->ResetEncoders();
		frictionTimer->Stop();
		frictionTimer->Reset();
		return true;
	}
	else{

		/*double averageDriveVel = drive->GetRightVelocity();
		printf("AVERAGE VEL: %f\n", averageDriveVel);
		if((averageDriveVel <= 0.005) && !(frictionTimer->Get() > 0.0)){
			frictionTimer->Start();
		}
		else if(!(averageDriveVel >= 0.0005) && (frictionTimer->Get() > 0.0)){
			frictionTimer->Stop();
			frictionTimer->Reset();
		}*/





		//finds difference b/w target and actual distance
		double distDifference = (distance - drive->GetAverageEncoderDistance());

		//based on that, determines what the drive speed should be so it ramps properly
		double adjDriveSpeed= drive->AutonRamping2(distDifference,MIN_STRAIGHT_DRIVE_SPEED, STEP1_STRAIGHT_DRIVE_SPEED,STEP2_STRAIGHT_DRIVE_SPEED,MAX_STRAIGHT_DRIVE_SPEED,MIN_STRAIGHT_DIST_DIFFERENCE,STEP1_STRAIGHT_DIST_DIFFERENCE,STEP2_STRAIGHT_DIST_DIFFERENCE);
		double ratio = adjDriveSpeed/MAX_STRAIGHT_DRIVE_SPEED;

		//uses gyro to correct
		double leftVal = adjDriveSpeed;
		double rightVal = adjDriveSpeed * RIGHT_DRIVE_CORRECTION;

		double yawError = drive->GetYaw()-startYaw; //since it is zeroed before this step starts, the angle IS the error

		//multiplies by ratio
		leftVal -= ratio * (yawError * STRAIGHT_P_CONSTANT);
		rightVal += ratio * (yawError * STRAIGHT_P_CONSTANT);

		printf("YawError %f \n", yawError);
		printf("Left Val %f \n", leftVal);
		printf("Right Val %f \n", rightVal);

		drive->TankDrive(-1.0*leftVal,-1.0*rightVal);
	}

	return false;

}
