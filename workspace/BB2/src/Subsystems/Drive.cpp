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

#define ACCEPTABLE_ANG 3.5
#define MIN_TURN_SPEED 0.23
#define MAX_TURN_SPEED 0.5

//Defined drive constants
#define INCHES_PER_TICK 0.004636 //measured 02-20-18
#define TICKS_PER_INCH 217.299549
#define TICKS_PER_DEGREE 1600/360
#define RESET_TIMEOUT 10

#define MAX_SPEED	1.0			// Limit Drive Speed to X%.
#define FF_MAX		2785

#define MAX_SPEED	1.0			// Limit Drive Speed to X%.

const static int PID_PRIMARY = 0;
//const static int PID_TURN = 1;


struct Gains {
	double kP, kI, kD, kF;
	double kIzone;
	double kPeakOutput;
};

//                                         kP   kI   kD   kF              Iz    PeakOut
constexpr static Gains kGains_MM =      { 0.3, 0.002, 30.0, 1023.0/FF_MAX,  400,  1.00 }; // measured 3200 max velocity


#define FF_MAX		2785
const uint8_t kTO = 10;

//Defined functions
#define max( A, B )				A > B ? A : B
#define min( A, B )				A < B ? A : B
#define LimitVal( L, T, H)		max( min( T, H ), L)

const uint8_t kTimeoutMs = 10;		// 10ms for timeouts.
const uint8_t kPIDLoopIdx = 0;

const double 	f = 1023/2785,		// Feed-Forward = 1023 Max Velocity / 3280nu(Max Native Units / 100ms measured).
				p = 0.3,		// Proportional
				i = 0.002, 		// Integral
				d = 30.0;		// Derivative

Drive::Drive(int frontLeftPort,int backLeftPort, int frontRightPort, int backRightPort, int pigeonPort) {
	//inits useful stuff
	pidgey = new PigeonIMU(pigeonPort);
	wasStraightButtonPressed = false;

    // Create & init all drive motors
	frontLeft = new WPI_TalonSRX(frontLeftPort);
	backLeft = new WPI_TalonSRX(backLeftPort);
	frontRight = new WPI_TalonSRX(frontRightPort);
	backRight = new WPI_TalonSRX(backRightPort);

	backLeft->Set(ControlMode::Follower, frontLeft->GetDeviceID());
	backRight->Set(ControlMode::Follower, frontRight->GetDeviceID());

	frontLeft->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
	frontRight->ConfigSelectedFeedbackSensor( FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0 );

	frontLeft->ConfigOpenloopRamp(0.1, 0.0);
	frontRight->ConfigOpenloopRamp(0.1, 0.0);

	double nuSp = 2.0 * 5.0 * 260.9;	// 3.5ft/s
	frontLeft->ConfigMotionAcceleration( nuSp * 1.5, 0 );
	frontLeft->ConfigMotionCruiseVelocity( nuSp, 0 );
	frontRight->ConfigMotionAcceleration( nuSp * 1.5, 0 );
	frontRight->ConfigMotionCruiseVelocity( nuSp, 0 );

	// Left PID
	frontLeft->Config_kF( PID_PRIMARY, kGains_MM.kF, kTO );
	frontLeft->Config_kP( PID_PRIMARY, kGains_MM.kP, kTO );
	frontLeft->Config_kI( PID_PRIMARY, kGains_MM.kI, kTO);
	frontLeft->Config_kD( PID_PRIMARY, kGains_MM.kD, kTO);
	frontLeft->Config_IntegralZone( PID_PRIMARY, kGains_MM.kIzone, kTO );
	frontLeft->ConfigPeakOutputForward(1.0, kTO );
	frontLeft->ConfigPeakOutputReverse(-1.0, kTO);
	frontLeft->SetInverted(true);
	backLeft->SetInverted(true);

	// Right PID
	frontRight->Config_kF( PID_PRIMARY, kGains_MM.kF, kTO );
	frontRight->Config_kP( PID_PRIMARY, kGains_MM.kP, kTO );
	frontRight->Config_kI( PID_PRIMARY, kGains_MM.kI, kTO);
	frontRight->Config_kD( PID_PRIMARY, kGains_MM.kD, kTO);
	frontRight->Config_IntegralZone( PID_PRIMARY, kGains_MM.kIzone, kTO );
	frontRight->ConfigPeakOutputForward(1.0, kTO );
	frontRight->ConfigPeakOutputReverse(-1.0, kTO);
	frontRight->SetInverted(false);
	backRight->SetInverted(false);


	// Status 10 provides the trajectory target for motion profile AND motion magic.
	frontLeft->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);
	frontRight->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);

	frontLeft->SetSensorPhase(true);
	frontRight->SetSensorPhase(true);

	turnTimer = new Timer();
	speed = 0;

	doneFlag = true;
	done2Flag = true;
}

void Drive::TankDrive(double left, double right){
	frontLeft->Set(ControlMode::PercentOutput, left);
	frontRight->Set(ControlMode::PercentOutput, right);

}

void Drive::BenzeneDrive(double throttle, double twist, bool driveStraightButton){
	float steer = 0.0;

	double heading = this->GetYaw();

	// If joystick thumb button for strait driving is pressed...
	if(driveStraightButton) {
		// On button being pressed, reset the gyro heading one time and use that as basis for driving straight.
		if(!wasStraightButtonPressed) {
			this->ResetYaw();
			wasStraightButtonPressed = true; //resets trigger
			heading = 0.0;
		}
		// Dump normal steering and twist inputs. Use gyro to drive strait instead.
		steer = dLimitVal( -0.5, heading * -0.05, 0.5 );
		twist = 0.0;
	}
	// Else, allow turning with joystick and twisting combined.
	else {
		// Else, apply deadband on steering and twist.
		if( fabs(twist) < 0.15 ) twist = 0.0;	// Deadband twist above 25%.
		steer += (twist * 0.50);		// Scale twist down by 50% then add to steer.
		wasStraightButtonPressed = false;				// Reset button press detect flag.
	}

	double leftSpeed = throttle + steer;		// Calculate left and right motor speeds.
	double rightSpeed = throttle - steer;

	this->TankDrive(leftSpeed, rightSpeed);


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
	this->FollowMode();
}

void Drive::NeutralizeDrive(){
	frontRight->NeutralOutput();
	frontLeft->NeutralOutput();

	frontRight->ConfigOpenloopRamp(0.1,kTO);
	frontLeft->ConfigOpenloopRamp(0.1,kTO);

	frontLeft->Set(ControlMode::PercentOutput,0.0);
	frontRight->Set(ControlMode::PercentOutput,0.0);
}

double Drive::dLimitVal(float low, float test,float high){
	if( test > high ) return high;

	if( test < low ) return low;
	return test;
}
void Drive::MotionMagicTurn(double targetAngle){
	double turnDist = targetAngle * TICKS_PER_DEGREE;
	frontLeft->Set(ControlMode::MotionMagic,turnDist);
	frontRight->Set(ControlMode::MotionMagic,-1.0*turnDist);
}

void Drive::ResetAccumulator(){
	frontLeft->SetIntegralAccumulator( 0.0, 0, 0 );
	frontRight->SetIntegralAccumulator( 0.0, 0, 0 );
}
void Drive::FollowMode(){
	backLeft->Set(ControlMode::Follower, frontLeft->GetDeviceID());
	backRight->Set(ControlMode::Follower, frontRight->GetDeviceID());
}
void Drive::TeleOpTurn(bool turnButton, bool override){
	double targetAngle = 180;
	double currentYaw = this->GetYaw();
	double angleError =fabs(currentYaw) - fabs(targetAngle);

	if(angleError >= 0){
		speed = (MAX_TURN_SPEED-MIN_TURN_SPEED) * pow(angleError,2)/(pow(angleError,2)+1300) + MIN_TURN_SPEED;
	}else {
		speed = -1.0*((MAX_TURN_SPEED-MIN_TURN_SPEED) * pow(angleError,2)/(pow(angleError,2)+1300) + MIN_TURN_SPEED);
	}

	if(turnButton){
		done2Flag = true;
	}else{
		done2Flag = false;
	}

	double leftSpeed = speed;
	double rightSpeed = -1.0 * speed;

	if(done2Flag){
		if(fabs(angleError) <= 1.0){
			this->TankDrive(0.0,0.0);
		}
		this->TankDrive(leftSpeed, rightSpeed);

		if(fabs(angleError) <= ACCEPTABLE_ANG){
			this->TankDrive(0.0,0.0);
		}else{
		this->TankDrive(leftSpeed, rightSpeed);
		}

		if(fabs(this->GetLeftVelocity()) <= .5){
			if(doneFlag){
			turnTimer->Start();
			doneFlag = false;
			}
		}else{
			doneFlag = true;
			turnTimer->Stop();
			turnTimer->Reset();
		}

		if(turnTimer->Get() >= 0.254){
			this->ResetYaw();
			this->NeutralizeDrive();
			turnTimer->Stop();
			printf("Done turning! \n");
			done2Flag = false;
		}
	}
	if(override){
		done2Flag = false;
	}
}


//*/
