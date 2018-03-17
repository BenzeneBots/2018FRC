/*
 *	BB1 - Benzene Bots FRC 2018 Robot Source Code
 *
 *
 * Arcade_Drive.h
 *
 *  Created on: Mar 14, 2018
 *      Author: James Kemp
 */

#ifndef ARCADE_DRIVE_H_
#define ARCADE_DRIVE_H_

// Return the float value, limited to the range low to high.
// ========================================================================
float fLimitVal( float low, float test, float hi ) {
	if( test > hi ) return hi;
	if( test < low ) return low;
	return test;
}

// Quickly set all four motors on the drivetrain.  Note, followers follow
// automatically.
// ========================================================================
void setDriveMtrSp( float mtrLeftSp, float mtrRightSp ) {
	// Speed is limited by "ConfigPeakOutputForward( MAX_SPEED, kTO )".
	mtrLMaster->Set( ControlMode::PercentOutput, mtrLeftSp );
	mtrRMaster ->Set( ControlMode::PercentOutput, mtrRightSp );
}

// ========================================================================
void ArcadeDrive() {
	float throttle=0.0, steer=0.0, twist=0;
	float lf=0.0, rt=0.0;
	static bool fBtnState = false;

	double heading = gyro->GetFusedHeading();

	//steer = joy->GetX( frc::GenericHID::JoystickHand::kRightHand );
	throttle = -1.0 * joy->GetY( frc::GenericHID::JoystickHand::kRightHand );
	twist = joy->GetTwist();

	// If joystick thumb button for strait driving is pressed...
	if( btnStrait->Get() ) {
		// On button being pressed, reset the gyro heading one time.
		if( fBtnState == false ) {
			gyro->SetFusedHeading( 0.0, 0 );
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




#endif /* ARCADE_DRIVE_H_ */
