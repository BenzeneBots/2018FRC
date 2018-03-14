/*
 *	BB1 - Benzene Bots FRC 2018 Robot Source Code
 *
 *
 */
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Robot.h>

PigeonIMU *imu;
TalonSRX *mtrLMaster, *mtrLSlave;
TalonSRX *mtrRMaster, *mtrRSlave;
Joystick *joy;
PigeonIMU *gyro;
JoystickButton *btnStrait;
JoystickButton *btnDrive;

#include <Path_Finder.h>
#include <Motion_Profile.h>
#include <Arcade_Drive.h>

class Robot : public TimedRobot {
private:
public:

	// ========================================================================
	void RobotInit() {
		imu = new PigeonIMU( 0 );
		mtrLMaster = new TalonSRX( 2 );
		mtrLSlave = new TalonSRX( 1 );
		mtrRMaster = new TalonSRX( 4 );
		mtrRSlave = new TalonSRX( 3 );
		joy = new Joystick( 1 );
    	btnStrait = new JoystickButton( joy, 2 );	// Thumb Button.
    	btnDrive = new JoystickButton( joy, 1);	// Trigger Button.
		gyro = new PigeonIMU( 0 );

		//mtrLMaster->Set( ControlMode::PercentOutput, 0.0 );
		//mtrRMaster->Set( ControlMode::PercentOutput, 0.0 );
		mtrLMaster->NeutralOutput();
		mtrRMaster->NeutralOutput();

		mtrLMaster->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
    	mtrLMaster->SetSelectedSensorPosition( 0, 0, 0 );
		mtrLMaster->SetSensorPhase(true);
    	mtrLMaster->SetInverted( false );
    	mtrLSlave->SetInverted( false );

    	// Right side motors get inverted!
    	mtrRMaster->ConfigSelectedFeedbackSensor( FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0 );
    	mtrRMaster->SetSelectedSensorPosition( 0, 0, 0 );
    	mtrRMaster->SetSensorPhase( true );
    	mtrRMaster->SetInverted( true );
    	mtrRSlave->SetInverted( true );

		// Left Side
		mtrLMaster->ConfigNominalOutputForward(0, kTimeoutMs);
		mtrLMaster->ConfigNominalOutputReverse(0, kTimeoutMs);
		mtrLMaster->ConfigPeakOutputForward(1, kTimeoutMs);
		mtrLMaster->ConfigPeakOutputReverse(-1, kTimeoutMs);
		mtrLMaster->Config_kF(kPIDLoopIdx, f,	kTimeoutMs);
		mtrLMaster->Config_kP(kPIDLoopIdx, p,   kTimeoutMs);	// (10% * 1023) / (350nu worst err measured).
		mtrLMaster->Config_kI(kPIDLoopIdx, i,   kTimeoutMs);	// Default: kP / 100
		mtrLMaster->Config_kD(kPIDLoopIdx, d,   kTimeoutMs);	// Default: kP * 10
		mtrLMaster->Config_IntegralZone( 0, 200, kTimeoutMs );

		// Right Side
		mtrRMaster->ConfigNominalOutputForward(0, kTimeoutMs);
		mtrRMaster->ConfigNominalOutputReverse(0, kTimeoutMs);
		mtrRMaster->ConfigPeakOutputForward(1, kTimeoutMs);
		mtrRMaster->ConfigPeakOutputReverse(-1, kTimeoutMs);
		mtrRMaster->Config_kF(kPIDLoopIdx, f,	kTimeoutMs);
		mtrRMaster->Config_kP(kPIDLoopIdx, p,   kTimeoutMs);	// (10% * 1023) / (350nu worst err measured).
		mtrRMaster->Config_kI(kPIDLoopIdx, i,   kTimeoutMs);	// Default: kP / 100
		mtrRMaster->Config_kD(kPIDLoopIdx, d,   kTimeoutMs);	// Default: kP * 10
		mtrRMaster->Config_IntegralZone( 0, 200, kTimeoutMs );

		// Setup Follower Slaves.
		mtrLSlave->Set( ControlMode::Follower, mtrLMaster->GetDeviceID() );
		mtrRSlave->Set( ControlMode::Follower, mtrRMaster->GetDeviceID() );

		// Status 10 provides the trajectory target for motion profile AND motion magic.
		mtrLMaster->SetStatusFramePeriod(
				StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);
		mtrRMaster->SetStatusFramePeriod(
				StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);

		// Calculate a path through waypoints.  Note, this may take a bit of
		// time if there are many waypoints!
		PathFinder();	// Updates leftTrajectory & leftTrajectory.

		mtrLMaster->ClearMotionProfileTrajectories();
		mtrRMaster->ClearMotionProfileTrajectories();

		// Profile uses 10 ms timing.
		TrajectoryDuration dt = TrajectoryDuration_10ms;
		mtrLMaster->ConfigMotionProfileTrajectoryPeriod( dt, kTimeoutMs);
		mtrRMaster->ConfigMotionProfileTrajectoryPeriod( dt, kTimeoutMs);

		// Run CAN at twice the speed of the profile so data xfer keeps up.
		mtrLMaster->ChangeMotionControlFramePeriod( TrajectoryDuration_10ms / 2 );
		mtrRMaster->ChangeMotionControlFramePeriod( TrajectoryDuration_10ms / 2 );

		gyro->SetFusedHeading( 0.0, 0 );	// Zero the gyro.

		// The mpThread handles shoveling trajectory points from the Top
		// API buffer out to the Talon(s).
		std::thread t1(mpThread);		// This starts the 100Hz thread right away.
    	t1.detach();					// Detach from this thread.
	}

	// ========================================================================
	void RobotPeriodic() {

	}

	// ========================================================================
	void DisabledInit() {
		mtrLMaster->NeutralOutput();
		mtrRMaster->NeutralOutput();
		enXfer = false;

		printf( "Disabled...\n" );
	}

	// ========================================================================
	void DisabledPeriodic() {
	}

	// ========================================================================
	void AutonomousInit() {
		printf( "AutoInit...\n" );
		//mtrLMaster->Set( ControlMode::MotionProfile, 0 );
		//mtrRMaster->Set( ControlMode::MotionProfile, 0 );

		mtrLMaster->ClearMotionProfileTrajectories();
		mtrRMaster->ClearMotionProfileTrajectories();

		// Make sure to reset the position to zero BEFORE starting the profile!
		mtrLMaster->SetSelectedSensorPosition( 0, 0, 0 );
		mtrRMaster->SetSelectedSensorPosition( 0, 0, 0 );
		gyro->SetFusedHeading( 0.0, 0 );

		mtrLMaster->ClearMotionProfileHasUnderrun( 0 );		// Clear any previous error.
		mtrRMaster->ClearMotionProfileHasUnderrun( 0 );

		// Fill the top buffer with Talon points.  Note, there is room for
		// about 2048 points for each Talon.  So, don't go too crazy!
		for (int i = 0; i < trajLen; ++i) {
			Segment s = leftTrajectory[i];
			double positionRot = s.position;
			double velocityRPM = s.velocity;

			TrajectoryPoint point;

			/* for each point, fill our structure and pass it to API */
			point.position = positionRot * 2607.6;  // Convert ft to nu.
			point.velocity = velocityRPM * 260.76; 	// Convert ft/s to nu/100ms
			point.headingDeg = 0; /* future feature - not used in this example*/
			point.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
			point.profileSlotSelect1 = 0; /* future feature  - not used in this example - cascaded PID [0,1], leave zero */
			point.timeDur = GetTrajectoryDuration((int) s.dt );

			// Set true on first point.
			point.zeroPos = (i == 0) ? true : false;
			// Set true on last point.
			point.isLastPoint = ((i+1) == trajLen) ? true : false;

			mtrLMaster->PushMotionProfileTrajectory( point );

			s = rightTrajectory[i];
			positionRot = s.position;
			velocityRPM = s.velocity;
			point.position = positionRot * 2607.6;  // Convert ft to nu.
			point.velocity = velocityRPM * 260.76; 	// Convert ft/s to nu/100ms

			mtrRMaster->PushMotionProfileTrajectory( point );
		}

		mtrLMaster->SetIntegralAccumulator( 0.0, 0, 0 );
		mtrRMaster->SetIntegralAccumulator( 0.0, 0, 0 );

		enXfer = true;
		printf( "Auto Pts Running...\n" );
	}

	// ========================================================================
	void AutonomousPeriodic() {
		static uint16_t cnt=0;

		if( enXfer ) {
			if( cnt > 5 ) {
				mtrLMaster->Set( ControlMode::MotionProfile, 1 );
				mtrRMaster->Set( ControlMode::MotionProfile, 1 );
			}
		}
		else {
			cnt = 0;
		}

		MotionProfileStatus mpStatus;
		mtrLMaster->GetMotionProfileStatus( mpStatus );

		if( enXfer && mpStatus.isLast ) {
			enXfer = false;
			cnt = 0;
			mtrLMaster->Set( ControlMode::MotionProfile, 2 );
			mtrRMaster->Set( ControlMode::MotionProfile, 2 );
			printf( "Motion Profile Finished\n" );

			if( mpStatus.hasUnderrun ) {
				printf( "\n****** Motion Profile UnderRun !!!!!!!\n\n" );
			}
		}

		if( mpStatus.isLast && cnt > 20 ) {
			mtrLMaster->Set( ControlMode::PercentOutput, 0.0 );
			mtrRMaster->Set( ControlMode::PercentOutput, 0.0 );
		}

		cnt += 1;

		SmartDashboard::PutNumber( "imuHeading", gyro->GetFusedHeading() );
		SmartDashboard::PutNumber( "chartOne", mtrLMaster->GetClosedLoopError(0) );
		SmartDashboard::PutNumber( "chartTwo", mtrLMaster->GetSelectedSensorVelocity(0) );
		SmartDashboard::PutNumber( "chartTwo", mtrLMaster->GetSelectedSensorPosition(0) );
	}

	// ========================================================================
	void TeleopInit() {
		printf( "TeleopInit...\n" );
		gyro->SetFusedHeading( 0.0, 0 );

		mtrLMaster->NeutralOutput();
		mtrRMaster->NeutralOutput();
	}

	// ========================================================================
	void TeleopPeriodic() {

	}

	// ========================================================================
	void TestInit() {
		setDriveMtrSp( 0.0, 0.0 );
		printf( "TestInit...\n" );
	}

	// ========================================================================
	void TestPeriodic() {
		ArcadeDrive();
		SmartDashboard::PutNumber( "imuHeading", gyro->GetFusedHeading() );

		// On joystick trigger, reset the gyro to zero.
		if( joy->GetRawButton( 1 ) ) gyro->SetFusedHeading( 0.0, 0 );

		SmartDashboard::PutNumber( "chartOne", mtrLMaster->GetClosedLoopError(0) );
		SmartDashboard::PutNumber( "chartTwo", mtrLMaster->GetSelectedSensorVelocity(0) / 100.0 );
		SmartDashboard::PutNumber( "chartTwo", mtrLMaster->GetSelectedSensorPosition(0) / 100.0 );
	}

};


START_ROBOT_CLASS(Robot)
