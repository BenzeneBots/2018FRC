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

#define kTimeoutMs 10
#define kPIDLoopIdx 0

#define F 0.3119	// Feed-Forward = 1023 Max Velocity / 3280nu(Max Native Units / 100ms measured).
#define P 0.3		// Proportional
#define I 0.0 		// Integral
#define	D 5.0


Drive::Drive(int frontLeftPort,int backLeftPort, int frontRightPort, int backRightPort, int pigeonPort) {
	pidgey = new PigeonIMU(pigeonPort);

    // Create all drive motors

	//talon code
	frontLeft = new WPI_TalonSRX(frontLeftPort);
	backLeft = new WPI_TalonSRX(backLeftPort);
	frontRight = new WPI_TalonSRX(frontRightPort);
	backRight = new WPI_TalonSRX(backRightPort);

	backLeft->Set(ControlMode::Follower, frontLeft->GetDeviceID());
	backRight->Set(ControlMode::Follower, frontRight->GetDeviceID());


	frontLeft->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,0); //sets the quad encoder as the primary sensor
	frontRight->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,0);

	drivetrain = new DifferentialDrive(*frontLeft, *frontRight);

	// Left Side
	frontLeft->ConfigNominalOutputForward(0, kTimeoutMs);
	frontLeft->ConfigNominalOutputReverse(0, kTimeoutMs);
	frontLeft->ConfigPeakOutputForward(1, kTimeoutMs);
	frontLeft->ConfigPeakOutputReverse(-1, kTimeoutMs);
	frontLeft->Config_kF(kPIDLoopIdx,F,kTimeoutMs);
	frontLeft->Config_kP(kPIDLoopIdx,P,kTimeoutMs);	// (10% * 1023) / (350nu worst err measured).
	frontLeft->Config_kI(kPIDLoopIdx,I,kTimeoutMs);	// Default: kP / 100
	frontLeft->Config_kD(kPIDLoopIdx,D,kTimeoutMs);	// Default: kP * 10
	frontLeft->Config_IntegralZone( 0, 200, kTimeoutMs );

	// Right Side
	frontRight->ConfigNominalOutputForward(0, kTimeoutMs);
	frontRight->ConfigNominalOutputReverse(0, kTimeoutMs);
	frontRight->ConfigPeakOutputForward(1, kTimeoutMs);
	frontRight->ConfigPeakOutputReverse(-1, kTimeoutMs);
	frontRight->Config_kF(kPIDLoopIdx,F,kTimeoutMs);
	frontRight->Config_kP(kPIDLoopIdx,P,kTimeoutMs);	// (10% * 1023) / (350nu worst err measured).
	frontRight->Config_kI(kPIDLoopIdx,I,kTimeoutMs);	// Default: kP / 100
	frontRight->Config_kD(kPIDLoopIdx,D,kTimeoutMs);	// Default: kP * 10
	frontRight->Config_IntegralZone( 0, 200, kTimeoutMs );

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
	double rawEncVal = -1.0 * this->GetRightEncoderValue();
	double rightDistanceVal = rawEncVal * INCHES_PER_TICK;
	return rightDistanceVal;
}

double Drive::GetLeftEncoderDistance(){//returns inches travelled by left side of drivetrain
	double rawEncVal = this->GetLeftEncoderValue();
	double leftDistanceVal = rawEncVal * INCHES_PER_TICK;
	return leftDistanceVal;
}

double Drive::GetRightVelocity(){//gets inches per second travelled by left side of drivetrain
	double rate = -1.0 * frontRight->GetSensorCollection().GetQuadratureVelocity();
	double VelocityVal = rate * INCHES_PER_TICK;
	return VelocityVal;
}

double Drive::GetLeftVelocity(){//gets inches per second travelled by right side of drivetrain
	double rate = frontLeft->GetSensorCollection().GetQuadratureVelocity();
	double VelocityVal = rate * INCHES_PER_TICK;
	return VelocityVal;
}

double Drive::GetRightEncoderValue(){
	double rightEncVal = frontRight->GetSensorCollection().GetQuadraturePosition();
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

double Drive::GetAverageVelocity(){
	double avgEncVel = (this->GetLeftVelocity() + this->GetRightVelocity())/2.0;
	return avgEncVel;
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
void Drive::AutonPrep(){
	frontLeft->ClearMotionProfileTrajectories();
	frontRight->ClearMotionProfileTrajectories();

	// Profile uses 20 ms timing.
	TrajectoryDuration dt = TrajectoryDuration_20ms;
	frontLeft->ConfigMotionProfileTrajectoryPeriod( dt, kTimeoutMs);
	frontRight->ConfigMotionProfileTrajectoryPeriod( dt, kTimeoutMs);

	// Run CAN at twice the speed of the profile so data xfer keeps up.
	frontLeft->ChangeMotionControlFramePeriod( TrajectoryDuration_20ms / 2 );
	frontRight->ChangeMotionControlFramePeriod( TrajectoryDuration_20ms / 2 );

	frontLeft->ClearMotionProfileTrajectories();
	frontRight->ClearMotionProfileTrajectories();

	// Make sure to reset the position to zero BEFORE starting the profile!
	frontLeft->SetSelectedSensorPosition( 0, 0, 0 );
	frontRight->SetSelectedSensorPosition( 0, 0, 0 );

	frontLeft->ClearMotionProfileHasUnderrun( 0 );		// Clear any previous error.
	frontRight->ClearMotionProfileHasUnderrun( 0 );

	frontLeft->SetIntegralAccumulator( 0.0, 0, 0 );
	frontRight->SetIntegralAccumulator( 0.0, 0, 0 );

	this->ResetYaw();
}
void Drive::LeftTraj(TrajectoryPoint point){
	frontLeft->PushMotionProfileTrajectory(point);
}
void Drive::RightTraj(TrajectoryPoint point2){
	frontRight->PushMotionProfileTrajectory(point2);
}
void Drive::AutonMotionProfile(int mode){
	frontLeft->Set( ControlMode::MotionProfile, mode);
	frontRight->Set( ControlMode::MotionProfile, mode);
}
void Drive::AutonStop(){
	frontLeft->Set( ControlMode::PercentOutput, 0.0 );
	frontRight->Set( ControlMode::PercentOutput, 0.0 );
}
void Drive::AutonProfileStatus(MotionProfileStatus status){
	frontLeft->GetMotionProfileStatus(status);
}
//*/
