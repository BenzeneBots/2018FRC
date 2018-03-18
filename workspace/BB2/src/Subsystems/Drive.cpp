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
#define TICKS_PER_INCH 217.299549
#define RESET_TIMEOUT 10

#define MAX_SPEED	1.0			// Limit Drive Speed to X%.
#define FF_MAX		2785

#define MAX_SPEED	1.0			// Limit Drive Speed to X%.
const static int REMOTE_0 = 0;
const static int REMOTE_1 = 1;
const static int PID_PRIMARY = 0;
const static int PID_TURN = 1;
const static int SLOT_0 = 0;
const static int SLOT_1 = 1;
const static int SLOT_2 = 2;
const static int SLOT_3 = 3;
struct Gains {
	double kP, kI, kD, kF;
	double kIzone;
	double kPeakOutput;
};

//                                         kP   kI   kD   kF              Iz    PeakOut
constexpr static Gains kGains_Distanc = { 0.1, 0.0,  0.0, 0.0,            100,  0.50 };
constexpr static Gains kGains_Turning = { 2.0, 0.0,  4.0, 0.0,            200,  1.00 };
constexpr static Gains kGains_Velocit = { 0.1, 0.0, 20.0, 1023.0/FF_MAX,  300,  0.50 }; // measured 3200 max velocity
constexpr static Gains kGains_MotProf = { 1.0, 0.0,  0.0, 1023.0/FF_MAX,  400,  1.00 }; // measured 3200 max velocity
constexpr static Gains kGains_MM =      { 0.4, 0.001, 4.0, 1023.0/FF_MAX,  400,  1.00 }; // measured 3200 max velocity

const static int kSlot_Distanc = SLOT_0;
const static int kSlot_Turning = SLOT_1;
const static int kSlot_Velocit = SLOT_2;
const static int kSlot_MotProf = SLOT_3;

#define FF_MAX		2785
const uint8_t kTO = 10;

//Defined functions
#define max( A, B )				A > B ? A : B
#define min( A, B )				A < B ? A : B
#define LimitVal( L, T, H)		max( min( T, H ), L)

const uint8_t kTimeoutMs = 10;		// 10ms for timeouts.
const uint8_t kPIDLoopIdx = 0;

const double 	f = 1023/2785,		// Feed-Forward = 1023 Max Velocity / 3280nu(Max Native Units / 100ms measured).
				p = 0.4,		// Proportional
				i = 0.001, 		// Integral
				d = 4.0;		// Derivative

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

	// Left PID
	frontLeft->Config_kF( PID_PRIMARY, kGains_MM.kF, kTO );
	frontLeft->Config_kP( PID_PRIMARY, kGains_MM.kP, kTO );
	frontLeft->Config_kI( PID_PRIMARY, kGains_MM.kI, kTO);
	frontLeft->Config_kD( PID_PRIMARY, kGains_MM.kD, kTO);
	frontLeft->Config_IntegralZone( PID_PRIMARY, kGains_MM.kIzone, kTO );
	frontLeft->SetInverted(false);
	backLeft->SetInverted(false);

	// Right PID
	frontRight->Config_kF( PID_PRIMARY, kGains_MM.kF, kTO );
	frontRight->Config_kP( PID_PRIMARY, kGains_MM.kP, kTO );
	frontRight->Config_kI( PID_PRIMARY, kGains_MM.kI, kTO);
	frontRight->Config_kD( PID_PRIMARY, kGains_MM.kD, kTO);
	frontRight->Config_IntegralZone( PID_PRIMARY, kGains_MM.kIzone, kTO );
	frontRight->SetInverted(true);
	backRight->SetInverted(true);

	// Setup Motion Magic values.
	double nuSp = 5.0 * 260.9;	// 2ft/s
	frontLeft->ConfigMotionAcceleration( nuSp * 1.5, 0 );
	frontLeft->ConfigMotionCruiseVelocity( nuSp, 0 );
	frontRight->ConfigMotionAcceleration( nuSp * 1.5, 0 );
	frontRight->ConfigMotionCruiseVelocity( nuSp, 0 );

	// Status 10 provides the trajectory target for motion profile AND motion magic.
	frontLeft->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);
	frontRight->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);
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
void Drive::MotionMagicStraight(double dist){
	double distance = dist*TICKS_PER_INCH;
	frontLeft->Set(ControlMode::MotionMagic,distance);
	frontRight->Set(ControlMode::MotionMagic,distance);
}
void Drive::NeutralizeDrive(){
	frontLeft->NeutralOutput();
	frontRight->NeutralOutput();
}
void Drive::setDriveMtrSp( float mtrLeftSpeed, float mtrRightSpeed) {
	frontLeft->Set(ControlMode::PercentOutput, mtrLeftSpeed);
	frontRight->Set(ControlMode::PercentOutput, mtrRightSpeed);
}

float Drive::fLimitVal(float low, float test,float high){
	if( test > high ) return high;
	if( test < low ) return low;
	return test;
}
void Drive::MyArcadeDrive(double throttle, double twist, bool driveStraightButton){
	float steer = 0.0;
	float lf=0.0, rt=0.0;
	static bool fBtnState = false;

	double heading = this->GetFusedHeading();

	// If joystick thumb button for strait driving is pressed...
	if(driveStraightButton) {
		// On button being pressed, reset the gyro heading one time.
		if( fBtnState == false ) {
			this->ResetFusedHeading();
			fBtnState = true;
			heading = 0.0;
			//cntThread = 0;		// Reset millisecond counter.
		}
		// Dump normal steering and twist inputs. Use gyro to drive strait instead.
		steer = fLimitVal( -0.5, heading * 0.05, 0.5 );
		twist = 0.0;
	}
	// Else, allow turning with joystick and twisting combined.
	else {
		// Else, apply deadband on steering and twist.
		if( fabs(steer) < 0.10 ) steer = 0.0;	// Must be at least 10% or above.
		steer *= 0.5;
		if( fabs(twist) < 0.15 ) twist = 0.0;	// Deadband twist above 25%.
		steer += (twist * 0.50);		// Scale twist down by 50% then add to steer.
		fBtnState = false;				// Reset button press detect flag.
	}

	lf = throttle + steer;		// Calculate left and right motor speeds.
	rt = throttle - steer;
	setDriveMtrSp( lf, rt);
}
//*/
