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
	mtrRMaster ->Set( ControlMode::PercentOutput, mtrRightSp );
}


//	On primary joystick, perform regular arcade style driving.
// ========================================================================
void ArcadeDrive( struct btns *b ) {
	double throttle=0.0, steer=0.0, twist=0, heading;
	float lf=0.0, rt=0.0;
	static double headingTar;

	#ifdef XBOX
		steer = joy->GetRawAxis( 4 );
		twist = 0.0;
		throttle = -1.0 * joy->GetY( frc::GenericHID::JoystickHand::kRightHand );
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
}




#endif /* ARCADE_DRIVE_H_ */
