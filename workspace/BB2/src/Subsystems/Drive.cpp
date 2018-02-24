/*
 * Drive.cpp
 *
 *  Created on: Jan 28, 2018
 *      Author: Murali
 */

#include <Subsystems/Drive.h>
#include <ctre/Phoenix.h>
#include <WPILib.h>
#include <Math.h>


//Defined drive constants
#define INCHES_PER_TICK 0.004636; //measured 02-20-18
#define RESET_TIMEOUT 10

//Defined functions
#define max( A, B )				A > B ? A : B
#define min( A, B )				A < B ? A : B
#define LimitVal( L, T, H)		max( min( T, H ), L)

Drive::Drive(int frontLeftPort,int backLeftPort, int frontRightPort, int backRightPort, int pigeonPort) {
	pidgey = new PigeonIMU(pigeonPort);

    // Create all drive motors

	//talon code
	frontLeft = new WPI_TalonSRX(frontLeftPort);
	backLeft = new WPI_TalonSRX(backLeftPort);
	frontRight = new WPI_TalonSRX(frontRightPort);
	backRight = new WPI_TalonSRX(backRightPort);

	//Practice Bot Code TODO disable later

	backLeft->Set(ControlMode::Follower, frontLeft->GetDeviceID());
	backRight->Set(ControlMode::Follower, frontRight->GetDeviceID());


	frontLeft->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,0); //sets the quad encoder as the primary sensor
	frontRight->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,0);

	drivetrain = new DifferentialDrive(*frontLeft, *frontRight);
}

void Drive::ArcadeDrive(double speed, double turn){//Drives the drivetrain based on
	drivetrain->ArcadeDrive(speed, turn, false);
}

void Drive::TankDrive(double left, double right){
	drivetrain->TankDrive(left, right, false);
}

double Drive::InputScale(double value,double power){

	if (value>0){
		return pow(value, power);
	}
	else{
		value = -1.0 * value;
		return -1.0*pow(value, power);
	}
}
void Drive::ResetEncoders(){//Resets Encoders to 0
	frontRight->GetSensorCollection().SetQuadraturePosition(0, 0); //should the timeoutMS be 0?
	frontLeft->GetSensorCollection().SetQuadraturePosition(0, 0);
}

double Drive::GetRightEncoderDistance(){//returns inches travelled by right side of drivetrain
	double rawEncVal = this->GetRightEncoderValue();
	double rightDistanceVal = rawEncVal * INCHES_PER_TICK;
	return rightDistanceVal;
}

double Drive::GetLeftEncoderDistance(){//returns inches travelled by left side of drivetrain
	double rawEncVal = this->GetLeftEncoderValue();
	double leftDistanceVal = rawEncVal * INCHES_PER_TICK;
	return leftDistanceVal;
}

double Drive::GetRightRate(){//gets inches per second travelled by left side of drivetrain
	double rate = frontRight->GetSensorCollection().GetQuadratureVelocity();
	double VelocityVal = rate * INCHES_PER_TICK;
	return VelocityVal;
}

double Drive::GetLeftRate(){//gets inches per second travelled by right side of drivetrain
	double rate = frontLeft->GetSensorCollection().GetQuadratureVelocity();
	double VelocityVal = rate * INCHES_PER_TICK;
	return VelocityVal;
}

double Drive::GetRightEncoderValue(){
	double rightEncVal = -1.0*frontRight->GetSensorCollection().GetQuadraturePosition();
	return rightEncVal;
}

double Drive::GetLeftEncoderValue(){
	double leftEncVal = frontLeft->GetSensorCollection().GetQuadraturePosition();
	return leftEncVal;
}

double Drive::GetAverageEncoderValue(){
	double avgEncVal = (this->GetRightEncoderValue() + this->GetLeftEncoderValue())/2.0;
	return avgEncVal;
}

double Drive::GetAverageEncoderDistance(){
	double avgEncDist = (this->GetRightEncoderDistance() + this->GetLeftEncoderDistance())/2.0;
	return avgEncDist;
}

void Drive::ResetYaw(){
	pidgey->SetYaw(0.0,RESET_TIMEOUT);
}
double Drive::GetYaw(){
	double ypr_array[3];
	pidgey->GetYawPitchRoll(ypr_array);
	return ypr_array[0];
}
void Drive::ResetFusedHeading(){
	pidgey->SetFusedHeading(0.0,RESET_TIMEOUT);
}
double Drive::GetFusedHeading(){
	return pidgey->GetFusedHeading();
}
void Drive::SetBrakeMode(){
	frontLeft->SetNeutralMode(NeutralMode::Brake);
	backLeft->SetNeutralMode(NeutralMode::Brake);
	frontRight->SetNeutralMode(NeutralMode::Brake);
	backRight->SetNeutralMode(NeutralMode::Brake);
}
void Drive::SetCoastMode(){
	frontLeft->SetNeutralMode(NeutralMode::Coast);
	backLeft->SetNeutralMode(NeutralMode::Coast);
	frontRight->SetNeutralMode(NeutralMode::Coast);
	backRight->SetNeutralMode(NeutralMode::Coast);
}

double Drive::NormalizeAngle(double angle){
	while (angle < 0){
		angle += 360.0;
	}
	while(angle > 359){
		angle -= 360.0;
	}
	return angle;
}

double Drive::AutonRamping1(double distDifference,double minDriveSpeed,double minDist, double rampRate,double maxDriveSpeed){
	if(distDifference>minDist){
		double autonDriveSpeed = -1.0/(pow(2.0,(rampRate*(distDifference - minDist - (log(maxDriveSpeed - minDriveSpeed)/(log(2.0)*rampRate))))))+maxDriveSpeed;
		return autonDriveSpeed;
	}
	else{
		return minDriveSpeed;
	}
}

double Drive::AutonRamping2(double distDifference,double minSpeed,double midSpeed1, double midSpeed2, double maxSpeed, double changeDist1, double changeDist2, double changeDist3){
	if(distDifference<changeDist1){
		return minSpeed;
	}
	else if(changeDist1<distDifference && distDifference<changeDist2){
		return midSpeed1;
	}
	else if(changeDist2<distDifference && distDifference<changeDist3){
		return midSpeed2;
	}
	else{
		return maxSpeed;
	}
}

//*/
