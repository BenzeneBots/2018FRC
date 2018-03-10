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

AutonDriveStraight::AutonDriveStraight(Drive *robotDrive, double dist) {
	drive = robotDrive;
	distance = dist;
	startYaw = 0;
}

AutonDriveStraight::~AutonDriveStraight() {
	// TODO Auto-generated destructor stub
}

void AutonDriveStraight::Initialize(){
	startYaw = drive->GetYaw();
}

bool AutonDriveStraight::Run(){
	if (drive->GetAverageEncoderDistance() >= distance){
		drive->TankDrive(0.0,0.0);
		drive->ResetEncoders();
		return true;
	}
	else{

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
