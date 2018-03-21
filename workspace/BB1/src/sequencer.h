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

#define delay( ms )		std::this_thread::sleep_for( std::chrono::microseconds( (ms)*1000 - 90 ) );

void seqThread( void );
bool flgAutoEn = false;
int ltDistNU, rtDistNU;

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
bool seqDwellOnMotion( double percent, int timeOut ) {
	while( 1 ) {
		mtrLMaster->Set( ControlMode::MotionMagic, ltDistNU );
		mtrRMaster->Set( ControlMode::MotionMagic, rtDistNU );
		// Dwell until X% of the distance is covered.
		if( fabs(mtrLMaster->GetSelectedSensorPosition(0)) > (fabs(ltDistNU) * percent) &&
				fabs(mtrRMaster->GetSelectedSensorPosition(0)) > (fabs(rtDistNU) * percent) )
			return false;		// Dist covered without timeout, return false.

		delay( 20 );

		timeOut -= 20;
		if( timeOut <= 0 ) {
			printf( "Error: Motion Magic Motion Timeout.\n" );
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
	LoadProfile( Mid_SwitchRight, false );
	//while( RunProfile() );			// Run the profile until completion.
	// Movement to switch done.
	// Deploy cube by reversing intake.
	mtrIntake->Set( OUTTAKE_SP );
	delay( CLAW_DEPLOY_TM );
	printf( "Stop intake.\n" );
	mtrIntake->Set( INTAKE_STOP );
	// Backup and rotate CCW to grab another cube.
	// Lower the claw at the same time.
	seqMotionMagic( -3.5, -1.0, 2.0, 3.0 );		seqDwellOnMotion( 0.85, 2000 );
	clawPick->Set( CLAW_LOWER );				delay( CLAW_LOWER_TM );
	clawClamp->Set( CLAW_OPEN );				delay( CLAW_OPEN_TM );
	seqMotionMagic( 2.0, 2.0, 2.0, 3.0 );		seqDwellOnMotion( 0.85, 2000 );
	clawClamp->Set( CLAW_CLOSE );				delay( CLAW_CLOSE_TM );
	clawPick->Set( CLAW_RAISE );				delay( CLAW_RAISE_TM );
	seqMotionMagic( -2.0, -2.0, 2.0, 3.0 );		seqDwellOnMotion( 0.85, 2000 );

	// Turn to the switch.
	delay( 1000 );
	// Deploy cube.
	mtrIntake->Set( OUTTAKE_SP );
	delay( CLAW_DEPLOY_TM );
	// Stop intake.
	mtrIntake->Set( INTAKE_STOP );
	delay( 20 );
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
			}
		}
		delay( 50 );
	//}
}

#endif /* SRC_SEQUENCER_H_ */
