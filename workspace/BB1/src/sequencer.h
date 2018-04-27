/*
 * sequencer.h
 *
 *  Created on: Mar 20, 2018
 *      Author: James Kemp
 */
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Robot.h>

#ifndef SRC_SEQUENCER_H_
#define SRC_SEQUENCER_H_

#define INTAKE_SP	-0.75
#define OUTTAKE_SP	1.00
#define INTAKE_STOP	0.0
#define INTAKE_HOLD -0.12

#define	CLAW_RAISE		DoubleSolenoid::kReverse
#define CLAW_LOWER		DoubleSolenoid::kForward
#define CLAW_NEUTRAL	DoubleSolenoid::kOff

#define CLAW_OPEN		true
#define CLAW_CLOSE		false

#define CLAW_DEPLAY_SM	400
#define CLAW_DEPLOY_TM	750		// Amount to time for cube to eject from intake.
#define CLAW_LOWER_TM	750		// Amount needed to lower the claw before opening.
#define CLAW_RAISE_TM	900
#define CLAW_OPEN_TM	100
#define CLAW_CLOSE_TM	100

//#define delay( ms )		std::this_thread::sleep_for( std::chrono::microseconds( (ms)*1000 - 90 ) );

void seqThread( void );
bool flgAutoEn = false;
int ltDistNU, rtDistNU;
AutonPathId pathIdx;

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
void seqMotionMagic( int lfDis, int rtDis, double vel, double accel) {

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

// Simply Sets the target of the elevator and moves it to the target
// ============================================================================
void seqMoveElevator( int height ){
	switch(height){
	case Ground:
		setElevatorPos( 0 );
		break;
	case SwitchHeight:
		setElevatorPos( 4000 );
		break;
	case ScaleHeight:
		setElevatorPos( 8000 );
		break;
	}

#ifndef PRACTICE_BOT
	DriveElevator( 0, mtrElavator , elevatorSW, &btns );
#endif
}

// Simply dwell until X percent of the Motion Magic motion is done.  Or, a timeout
// occurs - where timeout is given in milliseconds.  Returns false on no timeout.
// ============================================================================
bool seqDwellOnPosition( double percent, int timeOut ) {
	int lf, rt;
	int lfDis, rtDis;

	// Scale the distance we're happen with by a multiplier.
	lfDis = abs( ltDistNU ) * percent;
	rtDis = abs( rtDistNU ) * percent;

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

	// First, make sure the speed is above the threshold.
	while( 1 ) {
		// Get the absolute largest speed in ft/sec.
		vel = fabs( mtrLMaster->GetSelectedSensorVelocity(0) ) / 260.9;
		vel = fmax( (fabs( mtrRMaster->GetSelectedSensorVelocity(0) ) / 260.9), vel );

		if( vel > velThresh ) break;

		delay( 20 );
		timeOut -= 20;
		if( timeOut <= 0 ) {
			printf( "Error: dwell on motion never saw motion.\n" );
			return true;	// Return true that a timeout occured.
		}
	}

	// Next, wait until the speed falls back below the threshold.
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
void seqInit( AutonPathId idx ) {

	//pathIdx = _pathIdx;
	//sPos = _sPos;
	pathIdx = idx;

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
	LoadProfile( Mid_SwitchRight, true, false );
	while( RunProfile() ) delay( 20 );		// Run the profile until completion.
	seqDwellOnPosition( 0.85, 3000 );		// Wait for X% of position to be covered.

	// Movement to switch done.
	// Deploy cube by reversing intake.
	clawPick->Set( CLAW_LOWER );				delay( CLAW_DEPLAY_SM );
	clawPick->Set( CLAW_NEUTRAL );
	mtrIntake->Set( OUTTAKE_SP );				delay( CLAW_DEPLOY_TM );
	mtrIntake->Set( INTAKE_STOP );				delay( 500 );
	clawPick->Set( CLAW_RAISE );				delay( CLAW_DEPLOY_TM );

	// Load the named profile from the file-system into the RoboRIO API buffer.
	LoadProfile( Right_SwitchMid, true, true );
	while( RunProfile() ) delay( 20 );		// Run the profile until completion.
	seqDwellOnPosition( 0.85, 3000 );		// Wait for X% of position to be covered.

	clawPick->Set( CLAW_LOWER );					delay( 1200 );
	clawClamp->Set( CLAW_OPEN );

	mtrIntake->Set( INTAKE_SP );
	seqMotionMagic( 2.9, 2.9, 5, 10 );
	delay( 200  );
	seqDwellOnMotion( .02, 2000 );

	delay( 100 );

	clawClamp->Set( CLAW_CLOSE );					delay( 500 );
	mtrIntake->Set( INTAKE_HOLD );

	seqMotionMagic( -3.0, -3.0, 5, 10 );
	seqDwellOnMotion( .03, 2000 );

	clawPick->Set( CLAW_RAISE );
	mtrIntake->Set( INTAKE_STOP );


}

// ============================================================================
void seqMid_LeftSwitch() {
	printf( "Starting Mid Position to Left Switch sequence.\n" );
	gyro->SetFusedHeading( 0.0, 0 );	// Start with a zeroed gyro.

	// Load the named profile from the file-system into the RoboRIO API buffer.
	LoadProfile( Mid_SwitchLeft, true, false );
	while( RunProfile() ) delay( 20 );		// Run the profile until completion.
	seqDwellOnPosition( 0.85, 3000 );		// Wait for X% of position to be covered.

	// Deploy cube by reversing intake.
	clawPick->Set( CLAW_LOWER );				delay( CLAW_DEPLAY_SM );
	clawPick->Set( CLAW_NEUTRAL );

	mtrIntake->Set( OUTTAKE_SP );				delay( CLAW_DEPLOY_TM );
	mtrIntake->Set( INTAKE_STOP );				delay( 500 );

	LoadProfile( Left_SwitchMid, true, true );
	while( RunProfile() ) delay( 20 );		// Run the profile until completion.
	seqDwellOnPosition( 0.85, 3000 );		// Wait for X% of position to be covered.

	clawPick->Set( CLAW_LOWER );					delay( 1200 );
	clawClamp->Set( CLAW_OPEN );

	mtrIntake->Set( INTAKE_SP );
	seqMotionMagic( 3.0, 3.0, 5, 10 );
	delay( 200  );
	seqDwellOnMotion( .02, 2000 );

	delay( 100 );

	clawClamp->Set( CLAW_CLOSE );					delay( 500 );
	mtrIntake->Set( INTAKE_HOLD );

	seqMotionMagic( -3.0, -3.0, 5, 10 );
	seqDwellOnMotion( .03, 2000 );

	clawPick->Set( CLAW_RAISE );
	mtrIntake->Set( INTAKE_STOP );

}

// ============================================================================
void seqSide_Switch( bool invert ) {

	printf( "Starting Side Position to Switch sequence.\n" );
	gyro->SetFusedHeading( 0.0, 0 );	// Start with a zeroed gyro.

	// Load the named profile from the file-system into the RoboRIO API buffer.
	LoadProfile( Side_Switch, !invert, false );
	while( RunProfile() ) delay( 20 );		// Run the profile until completion.
	seqDwellOnPosition( 0.85, 3000 );		// Wait for X% of position to be covered.

	// Deploy cube by reversing intake.
	clawPick->Set( CLAW_LOWER );				delay( CLAW_DEPLAY_SM );
	clawPick->Set( CLAW_NEUTRAL );
	mtrIntake->Set( 0.70 );						delay( CLAW_DEPLOY_TM );
	mtrIntake->Set( INTAKE_STOP );
	clawPick->Set( CLAW_RAISE );

	LoadProfile( Switch_Cube, invert, true );
	while( RunProfile() ) delay( 20 );		// Run the profile until completion.

	clawPick->Set( CLAW_LOWER );					delay( 1200 );
	clawClamp->Set( CLAW_OPEN );

	mtrIntake->Set( INTAKE_SP );
	seqMotionMagic( 3.0, 3.0, 5, 10 );
	seqDwellOnMotion( .03, 2000 );

	delay( 200 );

	clawClamp->Set( CLAW_CLOSE );					delay( 500 );
	mtrIntake->Set( INTAKE_HOLD );

	clawPick->Set( CLAW_RAISE );
	mtrIntake->Set( INTAKE_STOP );
}


//	From side position to far scale.
// ============================================================================
void seqSide_ScaleFar( bool invert ) {

	printf( "Starting Side Position to Far Scale sequence.\n" );
	gyro->SetFusedHeading( 0.0, 0 );	// Start with a zeroed gyro.

	// Load the named profile from the file-system into the RoboRIO API buffer.
	LoadProfile( Side_ScaleFar, !invert, false );
	while( RunProfile() ) delay( 20 );		// Run the profile until completion.
	seqDwellOnPosition( 0.85, 3000 );		// Wait for X% of position to be covered.

	// Deploy cube by reversing intake.
	//clawPick->Set( CLAW_LOWER );				delay( CLAW_DEPLAY_SM );
	//clawPick->Set( CLAW_NEUTRAL );
	//mtrIntake->Set( 0.70 );						delay( CLAW_DEPLOY_TM );
	//mtrIntake->Set( INTAKE_STOP );
}

// ============================================================================
void seqSide_Scale( bool invert ) {
	printf( "Starting Side Position to Near Scale sequence.\n" );
	gyro->SetFusedHeading( 0.0, 0 );	// Start with a zeroed gyro.

	mtrLMaster->Set( ControlMode::PercentOutput, 0.0 );
	mtrRMaster->Set( ControlMode::PercentOutput, 0.0 );
	delay( 20 );

	// Load the named profile from the file-system into the RoboRIO API buffer.
	LoadProfile( Side_Scale, !invert, false );
	while( RunProfile() ) delay( 20 );		// Run the profile until completion.
	seqDwellOnPosition( 0.85, 3000 );		// Wait for X% of position to be covered.
	printf( "Run Profile Done.\n" );

	delay (500);

	seqMotionMagic( 1.0, 1.0, 5.0, 10.0 );
	printf( "Motion Magic Done.\n" );
	delay( 1000 );


	mtrElavator->ConfigPeakOutputForward( 0.7,	kTO );	// Peak Forward
	setElevatorPos( 10000 );
	int cnt = 0;
	mtrElavator->Set( ControlMode::Position, 15000 );
	while( mtrElavator->GetSelectedSensorPosition(0) < 14500 ) {
		//DriveElevator( 0, mtrElavator, elevatorSW, &btns );
		delay(20);
		if( ++cnt > 400 ) {
			printf( "Error: Timeout on elevator move!\n" );
			break;
		}
	}

	// Deploy cube by reversing intake.
	clawPick->Set( CLAW_LOWER );				delay( CLAW_DEPLAY_SM );
	clawPick->Set( CLAW_NEUTRAL );

	mtrIntake->Set( OUTTAKE_SP );				delay( CLAW_DEPLOY_TM );
	mtrIntake->Set( INTAKE_STOP );				delay( 500 );

	clawPick->Set( CLAW_RAISE );				delay( CLAW_DEPLAY_SM );

	setElevatorPos( 0 );
	mtrElavator->Set( ControlMode::Position, 0 );

	cnt = 0;
	while( mtrElavator->GetSelectedSensorPosition(0) > 500 ) {
		//DriveElevator( 0, mtrElavator, elevatorSW, &btns );
		delay(20);
		if( ++cnt > 200 )
			break;
	}

	mtrElavator->ConfigPeakOutputForward( 1.0,	kTO );	// Peak Forward

	/*
	double reverse = 1.0;
	if( invert ) reverse = -1.0;

	seqMotionMagic( reverse*-2.0, reverse*2.0, 4, 10 );
	seqDwellOnMotion( .03, 2000 );
	*/

}

void seqTestFunction() {
	printf( "Starting Test Function\n" );
	gyro->SetFusedHeading( 0.0, 0 );	// Start with a zeroed gyro.

	seqMotionMagic(-4.0,4.0,5,15);
}

// ============================================================================
void seqThread() {

	//while( 1 ) {
		// On Auto mode...
		if( frc::RobotState::IsAutonomous() && frc::RobotState::IsEnabled() ) {

			if( flgAutoEn ) {

				switch( pathIdx ) {
				case CenterLeftSwitch:
					seqMid_LeftSwitch();
					break;

				case CenterRightSwitch:
					seqMid_RightSwitch();
					break;

				case LeftSideSwitch:
					seqSide_Switch( true );
					break;

				case RightSideSwitch:
					seqSide_Switch( false );
					break;

				case LeftNearScale:
					seqSide_Scale( true );
					break;

				case RightNearScale:
					seqSide_Scale( false );
					break;

				case RightFarScale:
					seqSide_ScaleFar( false );
					break;

				case LeftFarScale:
					seqSide_ScaleFar( true );
					break;

				case DriveStraight:
					// Run 13 feet forward.  We should never do this!
					seqMotionMagic( 13.0, 13.0, 5.0, 10.0 );
					break;

				case TestFunction:
					printf( "Sequencer hit TestFunction. This is not what you want.\n" );
					seqTestFunction();
					break;

				default:
					printf( "Sequencer hit Default. This is not what you want.\n" );
					break;
				}


				//seqMid_RightSwitch();	// Auto Sequence: Mid Position to Right Switch

				//seqMid_LeftSwitch();

				//seqSide_Switch( true );

				// seqSide_ScaleFar( false );

				// seqSide_Scale( true );

				flgAutoEn = false;
				printf( "Auto Mode Done: %0.2f sec.\n", cntAuto * 0.005 );
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
