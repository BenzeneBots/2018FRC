/*
 * sequencer.h
 *
 *  Created on: Mar 20, 2018
 *      Author: James Kemp
 */

#ifndef SRC_SEQUENCER_H_
#define SRC_SEQUENCER_H_

#define INTAKE_SP	-0.75
#define OUTTAKE_SP	1.00
#define INTAKE_STOP	0.0

#define	CLAW_RAISE		DoubleSolenoid::kForward
#define CLAW_LOWER		DoubleSolenoid::kReverse
#define CLAW_NEUTRAL	DoubleSolenoid::kOff

#define CLAW_OPEN		true
#define CLAW_CLOSE		false

#define CLAW_DEPLOY_TM	750		// Amount to time for cube to eject from intake.
#define CLAW_LOWER_TM	750		// Amount needed to lower the claw before opening.
#define CLAW_RAISE_TM	900
#define CLAW_OPEN_TM	100
#define CLAW_CLOSE_TM	100

//#define delay( ms )		std::this_thread::sleep_for( std::chrono::microseconds( (ms)*1000 - 90 ) );

void seqThread( void );
bool flgAutoEn = false;
int ltDistNU, rtDistNU;

// This is a coarse delay function.  It only generates delays for auto mode.
// The minumum delay possible is 10ms.
// ============================================================================
void delay( int32_t ms ) {

	// Only delay if in auto mode and enabled.
	while( frc::RobotState::IsAutonomous() && frc::RobotState::IsEnabled() ) {
		ms -= 10;

		if( ms < 0 ) return;

		// Sleep 10 milliseconds.
		std::this_thread::sleep_for( std::chrono::microseconds( 10000 - 90 ) );
	}
}

// Given distance for left and right side, use Motion Magic to drive the motors
// until done.
//
//			   Units:
// Distance:	ft
// Velocity:	ft/sec
// Accel:		ft/sec^2
// ============================================================================
void seqMotionMagic( int lfDis, int rtDis, double vel, double accel ) {

	mtrLMaster->SetSelectedSensorPosition( 0, 0, 0 );
	mtrRMaster->SetSelectedSensorPosition( 0, 0, 0 );

	mtrLMaster->ConfigMotionCruiseVelocity( vel * 260.9, 0 );
	mtrRMaster->ConfigMotionCruiseVelocity( vel * 260.9, 0 );
	mtrLMaster->ConfigMotionAcceleration( accel * 260.9, 0 );
	mtrRMaster->ConfigMotionAcceleration( accel * 260.9, 0 );

	// Convert distance in feet to native units (4096cnt/rev).
	// Dis_nu = ft * 0.6366rot/ft * 4096nu/rot
	ltDistNU = lfDis * 2607;
	rtDistNU = rtDis * 2607;

	mtrLMaster->Set( ControlMode::MotionMagic, ltDistNU );
	mtrRMaster->Set( ControlMode::MotionMagic, rtDistNU );
}

// Simply dwell until X percent of the Motion Magic motion is done.  Or, a timeout
// occurs - where timeout is given in milliseconds.  Returns false on no timeout.
// ============================================================================
bool seqDwellOnPosition( double percent, int timeOut ) {
	int lf, rt;
	int lfDis, rtDis;

	// Scale the distance we're happen with by a multiplier.
	lfDis = fabs( ltDistNU ) * percent;
	rtDis = fabs( rtDistNU ) * percent;

	while( 1 ) {
		//mtrLMaster->Set( ControlMode::MotionMagic, ltDistNU );
		//mtrRMaster->Set( ControlMode::MotionMagic, rtDistNU );

		// Get the abs values of both encoders.
		lf = fabs(mtrLMaster->GetSelectedSensorPosition(0));
		rt = fabs(mtrRMaster->GetSelectedSensorPosition(0));

		// Dwell until X% of the distance is covered.
		if( (lf >= lfDis) && (rt >= rtDis) )
			return false;		// Dist covered without timeout, return false.

		delay( 20 );

		timeOut -= 20;
		if( timeOut <= 0 ) {
			printf( "Error: Dwell on position timeout.\n" );
			return true;		// Return true that a timeout happened.
		}
	}
}

// Dwell on motion on either side of the drivetrain above the threshold. Return
// true if a timeout happened.  Timeout given in milliseconds.
// ============================================================================
bool seqDwellOnMotion( double velThresh, int timeOut ) {
	double vel;

	while( 1 ) {
		// Get the absolute largest speed in ft/sec.
		vel = fabs( mtrLMaster->GetSelectedSensorVelocity(0) ) / 260.9;
		vel = fmax( (fabs( mtrRMaster->GetSelectedSensorVelocity(0) ) / 260.9), vel );
		if( vel <= velThresh )
			return false;	// No timeout.

		// Else...
		delay( 20 );
		timeOut -= 20;

		if( timeOut <= 0 ) {
			printf( "Error: Dwell on motion timeout.\n" );
			return true;		// Return true that a timeout happened.
		}
	}
}

// This just gets the sequencer thread running.
// ============================================================================
void seqInit( ) {

	std::thread t1( seqThread );	// This starts the thread right away.
	t1.detach();					// Detach from this thread.
	flgAutoEn = true;
}

// Enables or disables auto mode sequencer.
// ============================================================================
void enAutoSeq( bool flg ) {
	flgAutoEn = flg;
}

// Sequence from Mid Position to Right Switch.
// ============================================================================
void seqMid_RightSwitch() {
	//uint8_t tm = 250;

	printf( "Starting Mid Position to Right Switch sequence.\n" );
	gyro->SetFusedHeading( 0.0, 0 );	// Start with a zeroed gyro.
	
	// Load the named profile from the file-system into the RoboRIO API buffer.
	LoadProfile( Side_Scale, false );
	while( RunProfile() ) delay( 20 );		// Run the profile until completion.
	seqDwellOnPosition( 0.85, 3000 );		// Wait for X% of position to be covered.
/*
	// Movement to switch done.
	// Deploy cube by reversing intake.
	mtrIntake->Set( OUTTAKE_SP );				delay( CLAW_DEPLOY_TM );
	mtrIntake->Set( INTAKE_STOP );
	// Backup and rotate CCW to grab another cube.
	// Lower the claw at the same time.
	seqMotionMagic( -2.0, 1.5, 4.0, 3.0 );		seqDwellOnPosition( 0.75, 2000 );
	delay( 500 );
	clawPick->Set( CLAW_LOWER );				delay( CLAW_LOWER_TM );
	delay( 500 );
	clawClamp->Set( CLAW_OPEN );				delay( CLAW_OPEN_TM );
	mtrIntake->Set( INTAKE_SP );
	delay( 500 );
	seqMotionMagic( 2.0, 2.0, 6.0, 10.0 );		seqDwellOnPosition( 0.5, 2000 );
	clawClamp->Set( CLAW_CLOSE );				delay( CLAW_CLOSE_TM );
	mtrIntake->Set( INTAKE_STOP );
	delay( 100 );
	clawPick->Set( CLAW_RAISE );				delay( CLAW_RAISE_TM );
	delay( 250 );
	seqMotionMagic( -2.5, -2.5, 6.0, 10.0 );	seqDwellOnPosition( 0.85, 2000 );
	delay( 250 );

	printf( "Rot Degrees: %0.2f\n", gyro->GetFusedHeading() );
	double lf=0.1, rt=-0.1, heading;
	while( 1 ) {
		heading = gyro->GetFusedHeading();
		if( fabs(heading) < 5.0 ) {
			mtrLMaster->SetSelectedSensorPosition( 0, 0, 0 );
			mtrRMaster->SetSelectedSensorPosition( 0, 0, 0 );

			mtrLMaster->Set( ControlMode::Position, 0.0 );
			mtrRMaster->Set( ControlMode::Position, 0.0 );
			break;
		}
		if( heading > 0 ) {
			lf += 0.005;
			rt -= 0.005;
		}
		else {
			lf -= 0.005;
			rt += 0.005;
		}
		mtrLMaster->Set( ControlMode::PercentOutput, lf );
		mtrRMaster->Set( ControlMode::PercentOutput, rt );
		delay( 20 );
	}

	// Move forward and deploy.
	seqMotionMagic( 1.5, 1.5, 6.0, 10.0 );		seqDwellOnPosition( 0.85, 2000 );
	mtrIntake->Set( OUTTAKE_SP );				delay( CLAW_DEPLOY_TM );
	mtrIntake->Set( INTAKE_STOP );
*/
	printf( "Auto Mode Done.\n" );
}

// ============================================================================
void seqThread() {

	//while( 1 ) {
		// On Auto mode...
		if( frc::RobotState::IsAutonomous() && frc::RobotState::IsEnabled() ) {

			if( flgAutoEn ) {
				printf( "Starting auto mode...\n" );
				seqMid_RightSwitch();	// Auto Sequence: Mid Position to Right Switch
				flgAutoEn = false;
				delay( 500 );
			}
		}
		else {
			printf( "seqThread called in wrong mode." );
		}
		//delay( 50 );
	//}
}

#endif /* SRC_SEQUENCER_H_ */
