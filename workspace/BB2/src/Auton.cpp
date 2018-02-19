#include <ctre/Phoenix.h>
#include <WPILib.h>
#include <Subsystems/Drive.h>

#define AUTON_DRIVE_SPEED 0.5
#define LEFT_DRIVE_CORRECTION 1.0
#define INCHES_PER_TICK 0.00460194335

void AutonDriveStraight(double TargetDist, Drive *robotDrive){
	double DriveEncVal = (robotDrive->getLeftEncoderValue() + robotDrive->getRightEncoderValue())/2.0;

	if (INCHES_PER_TICK*DriveEncVal < TargetDist){
		robotDrive->TankDrive(AUTON_DRIVE_SPEED*LEFT_DRIVE_CORRECTION,AUTON_DRIVE_SPEED);
	}else{
		robotDrive->TankDrive(0.0,0.0);
	}
}
