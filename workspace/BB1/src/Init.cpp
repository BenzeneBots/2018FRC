/*
 * Init.cpp
 *
 *  Created on: Mar 26, 2018
 *      Author: James Kemp
 */

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Init.h>
#include <Robot.h>




//	Use this function to init both tele and test modes for the driver.
// ============================================================================
void DriverModeInit( Compressor *cp, TalonSRX *lf, TalonSRX *rt ) {
	cp->SetClosedLoopControl( COMPRESSOR );

	lf->ClearMotionProfileTrajectories();
	rt->ClearMotionProfileTrajectories();

	lf->NeutralOutput();
	rt->NeutralOutput();

	lf->ConfigOpenloopRamp( 0.1, kTO );
	rt->ConfigOpenloopRamp( 0.1, kTO );

	lf->Set( ControlMode::PercentOutput, 0.0 );
	rt->Set( ControlMode::PercentOutput, 0.0 );

	lf->SetNeutralMode( NeutralMode::Brake );
	rt->SetNeutralMode( NeutralMode::Brake );


	SmartDashboard::PutNumber( "chartOne", lf->GetSelectedSensorVelocity(0) );
	SmartDashboard::PutNumber( "chartTwo", 0.0 );
	SmartDashboard::PutNumber( "chartThree", 0.0 );		
}


//	Init the joysticks.
// ============================================================================
void JoystickInit( Joystick *joy, Joystick *joy2 ) {

}



//	Init everything for the pnumatics on the robot.
// ============================================================================
void PnumaticsInit( Compressor *cp, DoubleSolenoid *clawPick, Solenoid *clawClamp ) {
	clawPick->Set( DoubleSolenoid::kOff );
	clawClamp->Set( false );
}


// Setup the aux motors (Intake) on the robot.
// ============================================================================
void AuxMotorInit( Victor *mtrClimber, Victor *mtrIntake ) {

	mtrClimber->Set( 0.0 );
	mtrIntake->Set( 0.0 );
}


// Setup the PigeonIMU gyro.
// ============================================================================
void GyroInit( PigeonIMU *gyro ) {
	gyro->SetFusedHeading( 0.0, 0 );	// Reset gyro to a zero heading.
}

//	Everything to init the drivetrain motors goes here.
// ============================================================================
void DrivetrainInit( TalonSRX *lm, TalonSRX *ls, TalonSRX *rm, TalonSRX *rs ) {

	lm->NeutralOutput();
	rm->NeutralOutput();

	lm->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
	lm->SetSelectedSensorPosition( 0, 0, 0 );
	#ifdef PRACTICE_BOT
		lm->SetSensorPhase( true );
		lm->SetInverted( false );
		ls->SetInverted( false );
	#else
		lm->SetSensorPhase( true );
		lm->SetInverted( true );
		ls->SetInverted( true );
	#endif

	// Right side motors get inverted!
	rm->ConfigSelectedFeedbackSensor( FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0 );
	rm->SetSelectedSensorPosition( 0, 0, 0 );
	#ifdef PRACTICE_BOT
		rm->SetSensorPhase( true );
		rm->SetInverted( true );
		rs->SetInverted( true );
	#else
		rm->SetSensorPhase( true );
		rm->SetInverted( false );
		rs->SetInverted( false );
	#endif


	// Left Side
	lm->ConfigNominalOutputForward(0, kTO);
	lm->ConfigNominalOutputReverse(0, kTO);
	lm->Config_kF(kPIDLoopIdx, mpGains.ff,  kTO);
	lm->Config_kP(kPIDLoopIdx, mpGains.p,   kTO);
	lm->Config_kI(kPIDLoopIdx, mpGains.i,   kTO);
	lm->Config_kD(kPIDLoopIdx, mpGains.d,   kTO);
	lm->Config_IntegralZone( 0, mpGains.iZone, kTO );
	lm->ConfigPeakOutputForward( mpGains.peakOut, kTO );
	lm->ConfigPeakOutputReverse(-1 * mpGains.peakOut, kTO);

	// Right Side
	rm->ConfigNominalOutputForward(0, kTimeoutMs);
	rm->ConfigNominalOutputReverse(0, kTimeoutMs);
	rm->Config_kF(kPIDLoopIdx, mpGains.ff,  kTO);
	rm->Config_kP(kPIDLoopIdx, mpGains.p,   kTO);
	rm->Config_kI(kPIDLoopIdx, mpGains.i,   kTO);
	rm->Config_kD(kPIDLoopIdx, mpGains.d,   kTO);
	rm->Config_IntegralZone( 0, mpGains.iZone, kTO );
	rm->ConfigPeakOutputForward( mpGains.peakOut, kTO );
	rm->ConfigPeakOutputReverse(-1 * mpGains.peakOut, kTO);

	// Setup Follower Slaves.
	ls->Set( ControlMode::Follower, lm->GetDeviceID() );
	rs->Set( ControlMode::Follower, rm->GetDeviceID() );

	// Profile uses 10 ms timing.
	lm->ConfigMotionProfileTrajectoryPeriod( 10, kTimeoutMs );
	rm->ConfigMotionProfileTrajectoryPeriod( 10, kTimeoutMs );
	// Status 10 provides the trajectory target for motion profile AND motion magic.
	lm->SetStatusFramePeriod( StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);
	rm->SetStatusFramePeriod( StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);

	lm->ClearMotionProfileTrajectories();
	rm->ClearMotionProfileTrajectories();

	TrajectoryDuration dt = TrajectoryDuration_10ms;
	lm->ConfigMotionProfileTrajectoryPeriod( dt, kTimeoutMs);
	rm->ConfigMotionProfileTrajectoryPeriod( dt, kTimeoutMs);

	lm->ChangeMotionControlFramePeriod( TrajectoryDuration_10ms / 2 );
	rm->ChangeMotionControlFramePeriod( TrajectoryDuration_10ms / 2 );

	lm->ConfigClosedloopRamp( 0.1, kTO );
	rm->ConfigClosedloopRamp( 0.1, kTO );

	lm->SetNeutralMode( NeutralMode::Coast );
	rm->SetNeutralMode( NeutralMode::Coast );
}
