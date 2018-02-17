/*
 * Drive.cpp
 *
 *  Created on: Jan 28, 2018
 *      Author: Murali
 */

#include <Subsystems/Drive.h>
#include <ctre/Phoenix.h>
#include <WPILib.h>



//Defined drive constants
#define INCHES_PER_TICK 0.01410; //TODO measure this guy
#define POW 2.0;

//Defined functions
#define max( A, B )				A > B ? A : B
#define min( A, B )				A < B ? A : B
#define LimitVal( L, T, H)		max( min( T, H ), L)


namespace Drive {

Drive::Drive(int frontLeftPort,int backLeftPort, int frontRightPort, int backRightPort) {
	// TODO Auto-generated constructor stub

    // Create all drive motors

	//talon code
	frontLeft = new WPI_TalonSRX(frontLeftPort);
	backLeft = new WPI_TalonSRX(backLeftPort);
	frontRight = new WPI_TalonSRX(frontRightPort);
	backRight = new WPI_TalonSRX(backRightPort);

	backLeft->Set(ControlMode::Follower, frontLeft->GetDeviceID());
	backRight->Set(ControlMode::Follower, frontRight->GetDeviceID());

	frontLeft->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0); //sets the quad encoder as the primary sensor
	frontRight->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,0);

	drivetrain = new DifferentialDrive(*frontLeft, *frontRight);
}

void Drive::ArcadeDrive(double speed, double turn){//Drives the drivetrain based on
	drivetrain->ArcadeDrive(speed, turn, false);

}
double Drive::InputScale(double value, double power){
	if (value>=0){
		return pow(value, power);
	}
	else{
		return -1.0* pow(abs(value), power);
	}
}

void Drive::ResetEncoders(){//Resets Encoders to 0
	frontRight->GetSensorCollection().SetQuadraturePosition(0, 0); //should the timeoutMS be 0?
	frontLeft->GetSensorCollection().SetQuadraturePosition(0, 0);
}

double Drive::getRightEncoderDistance(){//returns inches travelled by right side of drivetrain
	double rawEncVal = frontRight->GetSensorCollection().GetQuadraturePosition();
	double rightDistanceVal = rawEncVal * INCHES_PER_TICK;
	return rightDistanceVal;
}

double Drive::getLeftEncoderDistance(){//returns inches travelled by left side of drivetrain
	double rawEncVal = frontLeft->GetSensorCollection().GetQuadraturePosition();
	double leftDistanceVal = rawEncVal * INCHES_PER_TICK;
	return leftDistanceVal;
}

double Drive::getRightRate(){//gets inches per second travelled by left side of drivetrain
	double rate = frontRight->GetSensorCollection().GetQuadratureVelocity();
	double VelocityVal = rate * INCHES_PER_TICK;
	return VelocityVal;
}

double Drive::getLeftRate(){//gets inches per second travelled by right side of drivetrain
	double rate = frontLeft->GetSensorCollection().GetQuadratureVelocity();
	double VelocityVal = rate * INCHES_PER_TICK;
	return VelocityVal;
}

double Drive::getRightEncoderValue(){
	double rightEncVal = frontRight->GetSensorCollection().GetQuadraturePosition();
	return rightEncVal;
}

double Drive::getLeftEncoderValue(){
	double leftEncVal = frontLeft->GetSensorCollection().GetQuadraturePosition();
	return leftEncVal;
}

}  /* namespace Drive */

//*/
