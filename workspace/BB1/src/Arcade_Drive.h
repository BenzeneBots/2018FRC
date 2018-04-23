/*
 * Arcade_Drive.h
 *
 *  Created on: Mar 14, 2018
 *      Author: James Kemp
 */

#ifndef ARCADE_DRIVE_H_
#define ARCADE_DRIVE_H_

#include <Robot.h>

#define MAX_SPEED	1.0	// Limit Drive Speed to X%.



/*
#ifdef PRACTICE_BOT
double fwdMult = -1.0;
#else
double fwdMult = 1.0;
#endif
*/

// Return the float value, limited to the range low to high.
// ========================================================================
float fLimitVal( float low, float test, float hi ) {
	if( test > hi ) return hi;
	if( test < low ) return low;
	return test;
}

// Quickly set all four motors on the drivetrain.
// ========================================================================
void setDriveMtrSp( float mtrLeftSp, float mtrRightSp ) {
	mtrLeftSp = fLimitVal( -MAX_SPEED, mtrLeftSp, MAX_SPEED );
	mtrLMaster->Set( ControlMode::PercentOutput, mtrLeftSp );

	mtrRightSp = fLimitVal( -MAX_SPEED, mtrRightSp, MAX_SPEED );
	mtrRMaster->Set( ControlMode::PercentOutput, mtrRightSp );
}

//	On primary joystick, perform 180 degree turn when button is pressed
// ========================================================================
void turn180 (struct btns *b) {

	static bool turningState = false;
	static double turnRevFactor = 1.0;

	if(b->btn[clockwiseTurn]){
		turningState = true ;
		turnRevFactor = 1.0 ;

		mtrLMaster->SetSelectedSensorPosition( 0, 0, 0 );
		mtrRMaster->SetSelectedSensorPosition( 0, 0, 0 );

		double vel = 5;
		double accel = 15;
		mtrLMaster->ConfigMotionCruiseVelocity( vel * 260.9, 0 );
		mtrRMaster->ConfigMotionCruiseVelocity( vel * 260.9, 0 );
		mtrLMaster->ConfigMotionAcceleration( accel * 260.9, 0 );
		mtrRMaster->ConfigMotionAcceleration( accel * 260.9, 0 );
		turnTimer->Reset();
		turnTimer->Start();
	}
	if(b->btn[counterTurn]){
		turningState = true ;
		turnRevFactor = -1.0 ;

		mtrLMaster->SetSelectedSensorPosition( 0, 0, 0 );
		mtrRMaster->SetSelectedSensorPosition( 0, 0, 0 );

		double vel = 5;
		double accel = 15;
		mtrLMaster->ConfigMotionCruiseVelocity( vel * 260.9, 0 );
		mtrRMaster->ConfigMotionCruiseVelocity( vel * 260.9, 0 );
		mtrLMaster->ConfigMotionAcceleration( accel * 260.9, 0 );
		mtrRMaster->ConfigMotionAcceleration( accel * 260.9, 0 );
		turnTimer->Reset();
		turnTimer->Start();
	}

	if(turningState){
		mtrLMaster->Set( ControlMode::MotionMagic, turnRevFactor * -10428.0 );
		mtrRMaster->Set( ControlMode::MotionMagic, turnRevFactor * 10428.0 );

		if((fabs(mtrLMaster->GetSensorCollection().GetQuadratureVelocity())<=150)&&(turnTimer->Get()>=0.5)){
			mtrLMaster->Set( ControlMode::PercentOutput, 0.0 );
			mtrRMaster->Set( ControlMode::PercentOutput, 0.0 );
			mtrLMaster->SetSelectedSensorPosition( 0, 0, 0 );
			mtrRMaster->SetSelectedSensorPosition( 0, 0, 0 );
			turnTimer->Stop();
			turningState = false;
		}
	}
}

//	On primary joystick, perform regular arcade style driving.
// ========================================================================
void ArcadeDrive( struct btns *b ) {
	double throttle=0.0, steer=0.0, twist=0, heading;
	float lf=0.0, rt=0.0;
	static double headingTar;
	//static bool isDriveReversed = false;
	static bool wasRevButtonPressed = false;
	static double driveRevFactor = 1.0;

	//updates drive reverse button stuff
	if(b->btn[reverse] && !wasRevButtonPressed){
		wasRevButtonPressed = true;
		driveRevFactor *= -1.0; //reverse drive direction
	}
	else if(!(b->btn[reverse])){
		wasRevButtonPressed = false;
	}

	#ifdef XBOX
		#ifdef PRACTICE_BOT
			steer = +1.0 * joy->GetRawAxis( 4 );
		#else
			steer = -1.0 * joy->GetRawAxis( 4 );
		#endif
		twist = 0.0;
		throttle = driveRevFactor * -1.0 * joy->GetY( frc::GenericHID::JoystickHand::kRightHand );
	#else
		// Only use joystick twist for steering.
		//steer = joy->GetX( frc::GenericHID::JoystickHand::kRightHand );
		throttle = -1.0 * joy->GetY( frc::GenericHID::JoystickHand::kRightHand );
		twist = joy->GetTwist();
	#endif

	// If joystick thumb button for strait driving is pressed...
	if( b->btn[ strait ] ) {
		heading = gyro->GetFusedHeading();
		// On button being pressed, capture the gyro heading as the target.
		if( b->btnPress[ strait ] ) headingTar = heading;
		double err = heading - headingTar;
		// Dump normal steering and twist inputs / Use gyro to drive strait instead.
		steer = fLimitVal( -0.5, err * 0.05, 0.5 );
		twist = 0.0;	// Don't allow twist inputs while in this mode.
	}
	// Else, allow turning with joystick and twisting combined.
	else {
		// Else, apply deadband on steering and twist.
		if( fabs(steer) < 0.10 ) steer = 0.0;	// Must be at least 10% or above.
		steer *= 0.5;
		if( fabs(twist) < 0.25 ) twist = 0.0;	// Deadband twist above 25%.
		steer += (twist * 0.50);		// Scale twist down by 50% then add to steer.
	}

	lf = throttle + steer;		// Calculate left and right motor speeds.
	rt = throttle - steer;
	setDriveMtrSp( lf, rt);


	turn180(b);
}




#endif /* ARCADE_DRIVE_H_ */
