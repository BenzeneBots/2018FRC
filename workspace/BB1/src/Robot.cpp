/*
 *	BB1 - Benzene Bots FRC 2018 Robot Source Code
 *
 *
 */
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Robot.h>
#include <CameraServer.h>

PigeonIMU *imu;
TalonSRX *mtrLMaster, *mtrLSlave;
TalonSRX *mtrRMaster, *mtrRSlave;
Victor *mtrIntake;

Joystick *joy;
Joystick *joy2;
PigeonIMU *gyro;
JoystickButton *btnStrait;
JoystickButton *btnIntake;
JoystickButton *btnOuttake;
JoystickButton *btnClawClose;
JoystickButton *btnClawOpen;

JoystickButton *btnIntake2;
JoystickButton *btnOuttake2;
JoystickButton *btnClawClose2;
JoystickButton *btnClawOpen2;

Compressor *airCompressor;
Solenoid *clawClamp;
DoubleSolenoid *clawPick;

#include <Path_Finder.h>
#include <Motion_Profile.h>
#include <Arcade_Drive.h>

const bool flgPID_MAGIC = false;
uint8_t stateAuto = 0;
uint16_t cntAuto = 0;

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
		mtrIntake = new Victor( 0 );
		joy = new Joystick( 1 );
		joy2 = new Joystick( 2 );

    	btnStrait = new JoystickButton( joy, 2 );	// Thumb Button.
    	btnIntake = new JoystickButton( joy, 1 );	// Trigger Button.
    	btnOuttake = new JoystickButton( joy, 7 );
    	btnClawOpen = new JoystickButton( joy, 3 );
    	btnClawClose = new JoystickButton( joy, 4 );

    	btnIntake2 = new JoystickButton( joy2, 1 );	// Trigger Button.
    	btnOuttake2 = new JoystickButton( joy2, 7 );
    	btnClawOpen2 = new JoystickButton( joy2, 3 );
    	btnClawClose2 = new JoystickButton( joy2, 4 );

    	gyro = new PigeonIMU( 0 );
		airCompressor = new Compressor(0);
		clawClamp = new Solenoid( 0 );
		clawPick = new DoubleSolenoid( 1, 2 );

		clawPick->Set( DoubleSolenoid::kOff );
		clawClamp->Set( false );

		mtrIntake->Set( 0.0 );

		airCompressor->SetClosedLoopControl( true );

		mtrLMaster->SetNeutralMode( NeutralMode::Coast );
		mtrLMaster->NeutralOutput();
		mtrRMaster->SetNeutralMode( NeutralMode::Coast );
		mtrRMaster->NeutralOutput();

		// Setup Left Feedback Stuff
		mtrLMaster->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
    	mtrLMaster->SetSelectedSensorPosition( 0, 0, 0 );
		mtrLMaster->SetSensorPhase(true);
    	mtrLMaster->SetInverted( false );
    	mtrLSlave->SetInverted( false );
		mtrLMaster->ConfigPeakOutputForward( MAX_SPEED, kTO );
		mtrLMaster->ConfigPeakOutputReverse( -MAX_SPEED, kTO );

    	/*

		if( flgPID_MAGIC ) {
			mtrLMaster->ConfigRemoteFeedbackFilter( gyro->GetDeviceNumber(),
					RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw,
					1,	// Filter #
					kTimeoutMs );
			mtrLMaster->ConfigSensorTerm( SensorTerm::SensorTerm_Sum0,
					FeedbackDevice::CTRE_MagEncoder_Relative, kTimeoutMs );
			mtrLMaster->ConfigSensorTerm( SensorTerm::SensorTerm_Sum1,
					FeedbackDevice::RemoteSensor0, kTimeoutMs );
		}

		*/

		// Setup Right Feedback Stuff - Right side motors get inverted!
    	mtrRMaster->ConfigSelectedFeedbackSensor( FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0 );
    	mtrRMaster->SetSelectedSensorPosition( 0, 0, 0 );
    	mtrRMaster->SetSensorPhase( true );
    	mtrRMaster->SetInverted( true );
    	mtrRSlave->SetInverted( true );
		mtrRMaster->ConfigPeakOutputForward( MAX_SPEED, kTO );
		mtrRMaster->ConfigPeakOutputReverse( -MAX_SPEED, kTO );

		// Left PID
		mtrLMaster->Config_kF( PID_PRIMARY, kGains_MM.kF, kTO );
		mtrLMaster->Config_kP( PID_PRIMARY, kGains_MM.kP, kTO );
		mtrLMaster->Config_kI( PID_PRIMARY, kGains_MM.kI, kTO);
		mtrLMaster->Config_kD( PID_PRIMARY, kGains_MM.kD, kTO);
		mtrLMaster->Config_IntegralZone( PID_PRIMARY, kGains_MM.kIzone, kTO );

		// Right PID
		mtrRMaster->Config_kF( PID_PRIMARY, kGains_MM.kF, kTO );
		mtrRMaster->Config_kP( PID_PRIMARY, kGains_MM.kP, kTO );
		mtrRMaster->Config_kI( PID_PRIMARY, kGains_MM.kI, kTO);
		mtrRMaster->Config_kD( PID_PRIMARY, kGains_MM.kD, kTO);
		mtrRMaster->Config_IntegralZone( PID_PRIMARY, kGains_MM.kIzone, kTO );

		// Setup Motion Magic values.
		double nuSp = 5.0 * 260.9;	// 2ft/s
		mtrLMaster->ConfigMotionAcceleration( nuSp * 1.5, 0 );
		mtrLMaster->ConfigMotionCruiseVelocity( nuSp, 0 );
		mtrRMaster->ConfigMotionAcceleration( nuSp * 1.5, 0 );
		mtrRMaster->ConfigMotionCruiseVelocity( nuSp, 0 );

		/*
		if( flgPID_MAGIC ) {
			mtrRMaster->ConfigRemoteFeedbackFilter( gyro->GetDeviceNumber(),
					RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw,
					1,	// Filter #
					kTimeoutMs );
			mtrRMaster->ConfigSensorTerm( SensorTerm::SensorTerm_Sum0,
					FeedbackDevice::CTRE_MagEncoder_Relative, kTimeoutMs );
			mtrRMaster->ConfigSensorTerm( SensorTerm::SensorTerm_Diff1,
					FeedbackDevice::RemoteSensor0, kTimeoutMs );
		}

    	// Left Side PID Values
		mtrLMaster->ConfigNominalOutputForward( 0.0, kTO );
		mtrLMaster->ConfigNominalOutputReverse( 0.0, kTO );
		mtrLMaster->ConfigPeakOutputForward( 1, kTO );
		mtrLMaster->ConfigPeakOutputReverse( -1, kTO );
		mtrLMaster->Config_kF( PID_PRIMARY, kGains_MotProf.kF, kTO );
		mtrLMaster->Config_kP( PID_PRIMARY, kGains_MotProf.kP, kTO );	// Default: 10% of FF
		mtrLMaster->Config_kI( PID_PRIMARY, kGains_MotProf.kI, kTO);	// Default: kP / 100
		mtrLMaster->Config_kD( PID_PRIMARY, kGains_MotProf.kD, kTO);	// Default: kP * 10
		mtrLMaster->Config_IntegralZone( PID_PRIMARY, kGains_MotProf.kIzone, kTO );
		mtrLMaster->ConfigPeakOutputForward( kGains_MotProf.kPeakOutput, kTO );
		mtrLMaster->ConfigPeakOutputReverse( kGains_MotProf.kPeakOutput, kTO );

		if( flgPID_MAGIC ) {
			// Setup secondary PID loop on remote feedback sensor (gyro).
			mtrLMaster->Config_kF( PID_TURN, kGains_Turning.kF, kTO );
			mtrLMaster->Config_kP( PID_TURN, kGains_Turning.kP, kTO );
			mtrLMaster->Config_kI( PID_TURN, kGains_Turning.kI, kTO);
			mtrLMaster->Config_kD( PID_TURN, kGains_Turning.kD, kTO);
			mtrLMaster->Config_IntegralZone( PID_TURN, kGains_Turning.kIzone, kTO );
		}

		// Right Side PID Values
		mtrRMaster->ConfigNominalOutputForward( 0.0, kTO );
		mtrRMaster->ConfigNominalOutputReverse( 0.0, kTO );
		mtrRMaster->ConfigPeakOutputForward( 1, kTO );
		mtrRMaster->ConfigPeakOutputReverse( -1, kTO );
		mtrRMaster->Config_kF( PID_PRIMARY, kGains_MotProf.kF, kTO );
		mtrRMaster->Config_kP( PID_PRIMARY, kGains_MotProf.kP, kTO );	// Default: 10% of FF
		mtrRMaster->Config_kI( PID_PRIMARY, kGains_MotProf.kI, kTO);	// Default: kP / 100
		mtrRMaster->Config_kD( PID_PRIMARY, kGains_MotProf.kD, kTO);	// Default: kP * 10
		mtrRMaster->Config_IntegralZone( PID_PRIMARY, kGains_MotProf.kIzone, kTO );
		mtrRMaster->ConfigPeakOutputForward( kGains_MotProf.kPeakOutput, kTO );
		mtrRMaster->ConfigPeakOutputReverse( kGains_MotProf.kPeakOutput, kTO );


		if( flgPID_MAGIC ) {
			// Setup secondary PID loop on remote feedback sensor (gyro).
			mtrRMaster->Config_kF( PID_TURN, kGains_Turning.kF, kTO );
			mtrRMaster->Config_kP( PID_TURN, kGains_Turning.kP, kTO );
			mtrRMaster->Config_kI( PID_TURN, kGains_Turning.kI, kTO);
			mtrRMaster->Config_kD( PID_TURN, kGains_Turning.kD, kTO);
			mtrRMaster->Config_IntegralZone( PID_TURN, kGains_Turning.kIzone, kTO );
		}

		if( flgPID_MAGIC ) {
			int closedLoopTimeMs = 1;
			mtrRMaster->ConfigSetParameter(ParamEnum::ePIDLoopPeriod, closedLoopTimeMs, 0x00, PID_PRIMARY, kTO );
			mtrRMaster->ConfigSetParameter(ParamEnum::ePIDLoopPeriod, closedLoopTimeMs, 0x00, PID_TURN, kTO);

			// The right side uses an inverted version of the gyro.
			mtrRMaster->ConfigAuxPIDPolarity( false, kTO );
		}

		*/

		// Setup Follower motors on drivetrain.
		mtrLSlave->Set( ControlMode::Follower, mtrLMaster->GetDeviceID() );
		mtrRSlave->Set( ControlMode::Follower, mtrRMaster->GetDeviceID() );

		/*
		// Select slots for primary and secondary PID loops.
		mtrLMaster->SelectProfileSlot( kSlot_MotProf, PID_PRIMARY );
		mtrRMaster->SelectProfileSlot( kSlot_MotProf, PID_PRIMARY );
		if( flgPID_MAGIC ) {
			mtrLMaster->SelectProfileSlot( kSlot_Turning, PID_TURN );
			mtrRMaster->SelectProfileSlot( kSlot_Turning, PID_TURN );
		}

		// Motion Profile uses 10ms timing.
		mtrLMaster->ConfigMotionProfileTrajectoryPeriod( 10, kTimeoutMs );
		mtrRMaster->ConfigMotionProfileTrajectoryPeriod( 10, kTimeoutMs );
		// Status 10 provides the trajectory target for motion profile AND motion magic.
		mtrLMaster->SetStatusFramePeriod(
				StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);
		mtrRMaster->SetStatusFramePeriod(
				StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);

		// Calculate a path through waypoints.  Note, this may take a bit of
		// time if there are many waypoints!
		PathFinder();	// Updates leftTrajectory & leftTrajectory with malloc data.

		// Clear any existing MP trajectories just in case...
		mtrLMaster->ClearMotionProfileTrajectories();
		mtrRMaster->ClearMotionProfileTrajectories();

		// Cranks up the CAN bus, I think.
		TrajectoryDuration dt = TrajectoryDuration_10ms;
		mtrLMaster->ConfigMotionProfileTrajectoryPeriod( dt, kTimeoutMs);
		mtrRMaster->ConfigMotionProfileTrajectoryPeriod( dt, kTimeoutMs);

		mtrLMaster->ChangeMotionControlFramePeriod( TrajectoryDuration_10ms / 2 );
		mtrRMaster->ChangeMotionControlFramePeriod( TrajectoryDuration_10ms / 2 );
		*/

		gyro->SetFusedHeading( 0.0, 0 );

		// The mpThread handles shoveling trajectory points from the Top
		// API buffer out to the Talon(s).
		std::thread t1(mpThread);		// This starts the 100Hz thread right away.
    	t1.detach();					// Detach from this thread.

    	CameraServer::GetInstance()->StartAutomaticCapture();
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
		/*
		//mtrLMaster->Set( ControlMode::MotionProfile, 0 );
		//mtrRMaster->Set( ControlMode::MotionProfile, 0 );

		mtrLMaster->ClearMotionProfileTrajectories();
		mtrRMaster->ClearMotionProfileTrajectories();

		mtrLMaster->SetSelectedSensorPosition( 0, 0, 0 );
		mtrRMaster->SetSelectedSensorPosition( 0, 0, 0 );
		gyro->SetFusedHeading( 0.0, 0 );

		mtrLMaster->ClearMotionProfileHasUnderrun( 0 );
		mtrRMaster->ClearMotionProfileHasUnderrun( 0 );

		// Fill the top buffer with Talon points.  Note, there is room for
		// about 2048 points for each Talon.  So, don't go too crazy!
		for (int i = 0; i < trajLen; ++i) {
			Segment s = leftTrajectory[i];
			double positionRot = s.position;
			double velocityRPM = s.velocity;

			TrajectoryPoint point;

			// for each point, fill our structure and pass it to API
			point.position = positionRot * 2607.6;  // Convert ft to nu.
			point.velocity = velocityRPM * 260.76; 	// Convert ft/s to nu/100ms
			point.headingDeg = 0; // future feature - not used in this example
			point.profileSlotSelect0 = 0; // which set of gains would you like to use [0,3]?
			point.profileSlotSelect1 = 0; // future feature  - not used in this example - cascaded PID [0,1], leave zero
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
		*/
    	mtrLMaster->SetSelectedSensorPosition( 0, 0, 0 );
    	mtrRMaster->SetSelectedSensorPosition( 0, 0, 0 );
		gyro->SetFusedHeading( 0.0, 0 );
		stateAuto = 0;
		cntAuto = 0;
	}

	// ========================================================================
	void AutonomousPeriodic() {

		/*
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
		*/

		double heading = gyro->GetFusedHeading();

		// Strait
		if( stateAuto == 0 ) {
			if( cntAuto == 1 ) {
				printf("Auto Starting...\n" );
			}
			int dist = (5 * 0.63662) * 4096;
			mtrLMaster->Set( ControlMode::MotionMagic, dist );
			mtrRMaster->Set( ControlMode::MotionMagic, dist );
			if( mtrLMaster->GetSelectedSensorVelocity( 0 ) < 100 ) {
				if( cntAuto > 50 ) {
					stateAuto += 1;
					cntAuto = 0;
				}
			}
			else {
				cntAuto = 2;
			}
		}
		// Turn
		else if( stateAuto == 1 ) {
			if( cntAuto == 1 ) {
				printf( "Turn pre heading err: %0.2f\n", heading );
		    	mtrLMaster->SetSelectedSensorPosition( 0, 0, 0 );
		    	mtrRMaster->SetSelectedSensorPosition( 0, 0, 0 );
			}
			int dist = 16000 / 4;
			mtrLMaster->Set( ControlMode::MotionMagic, dist * -1 );
			mtrRMaster->Set( ControlMode::MotionMagic, dist );
			if( mtrLMaster->GetSelectedSensorVelocity( 0 ) < 100 ) {
				if( cntAuto > 50 ) {
					stateAuto += 1;
					cntAuto = 0;
				}
			}
			else {
				cntAuto = 2;
			}
		}
		// Strait
		else if ( stateAuto == 2 ) {
			if( cntAuto == 1 ) {
				printf("Last strait.\n" );
		    	mtrLMaster->SetSelectedSensorPosition( 0, 0, 0 );
		    	mtrRMaster->SetSelectedSensorPosition( 0, 0, 0 );
			}
			int dist = (5 * 0.63662) * 4096;
			mtrLMaster->Set( ControlMode::MotionMagic, dist );
			mtrRMaster->Set( ControlMode::MotionMagic, dist );
			if( mtrLMaster->GetSelectedSensorVelocity( 0 ) < 100 ) {
				if( cntAuto > 50 ) {
					stateAuto += 1;
					cntAuto = 0;
				}
			}
			else {
				cntAuto = 2;
			}
		}
		else if ( stateAuto == 3 ) {
			printf( "Auto Done\n" );
			stateAuto += 1;
		}

		SmartDashboard::PutNumber( "imuHeading", heading );
		SmartDashboard::PutNumber( "chartOne", heading * 1000. );
		SmartDashboard::PutNumber( "chartTwo", mtrLMaster->GetSelectedSensorVelocity(0) * 10. );
		SmartDashboard::PutNumber( "chartThree", mtrLMaster->GetSelectedSensorPosition(0) );

		cntAuto += 1;
	}

	// ========================================================================
	void TeleopInit() {
		printf( "TeleopInit...\n" );
		gyro->SetFusedHeading( 0.0, 0 );
		mtrLMaster->SetSelectedSensorPosition( 0, 0, 0 );
		mtrLMaster->SetSelectedSensorPosition( 0, 0, 0 );
		airCompressor->SetClosedLoopControl( true);

		mtrLMaster->NeutralOutput();
		mtrRMaster->NeutralOutput();
		SmartDashboard::PutNumber( "chartOne", mtrLMaster->GetSelectedSensorVelocity(0) );
		SmartDashboard::PutNumber( "chartTwo", 0.0 );
		SmartDashboard::PutNumber( "chartThree", 0.0 );
	}

	// ========================================================================
	void TeleopPeriodic() {
		static uint16_t cnt=0;

		airCompressor->SetClosedLoopControl( true);

		ArcadeDrive();

		if( btnClawOpen->Get() || btnClawOpen2->Get() ) {
			clawClamp->Set( true );
		}
		if( btnClawClose->Get() || btnClawClose2->Get() ) {
			clawClamp->Set( false );
		}

		// Push POV up to raise claw.
		if( (joy->GetPOV(0) == 180) ||  (joy2->GetPOV(0) == 180) ) {
			clawPick->Set( DoubleSolenoid::kForward );
		}
		// Push POV down to lower claw.
		else if( (joy->GetPOV(0) == 0) || (joy2->GetPOV(0) == 0) ) {
			clawPick->Set( DoubleSolenoid::kReverse );
		}
		// On no POV pushed, stop the claw mid way up/down.
		else {
			clawPick->Set( DoubleSolenoid::kOff );
		}

		if( btnIntake->Get() || btnIntake2->Get() )
			mtrIntake->Set( (joy->GetThrottle() + 1.0) / 2.0 );
		else if ( btnOuttake->Get() || btnOuttake2->Get() )
			mtrIntake->Set( -1.0 );
		else
			mtrIntake->Set( 0.0 );

		// On joystick trigger, reset the gyro to zero.
		if( joy->GetRawButton( 1 ) ) gyro->SetFusedHeading( 0.0, 0 );

		//SmartDashboard::PutNumber( "chartOne", gyro->GetFusedHeading() );

		if( ++cnt > 30 ) {
			int lf = mtrLMaster->GetSelectedSensorPosition( 0 );
			int rt = mtrLMaster->GetSelectedSensorPosition( 0 );
			printf( "%d   %d   %0.2f\n", lf, rt, gyro->GetFusedHeading() );
			cnt = 0;
		}
	}

	// ========================================================================
	void TestInit() {
		setDriveMtrSp( 0.0, 0.0 );
		airCompressor->SetClosedLoopControl( true);
		printf( "TestInit...\n" );
	}

	// ========================================================================
	void TestPeriodic() {
		airCompressor->SetClosedLoopControl( true);

		ArcadeDrive();

		if( btnClawOpen->Get() || btnClawOpen2->Get() ) {
			clawClamp->Set( true );
		}
		if( btnClawClose->Get() || btnClawClose2->Get() ) {
			clawClamp->Set( false );
		}

		// Push POV up to raise claw.
		if( (joy->GetPOV(0) == 180) ||  (joy2->GetPOV(0) == 180) ) {
			clawPick->Set( DoubleSolenoid::kForward );
		}
		// Push POV down to lower claw.
		else if( (joy->GetPOV(0) == 0) || (joy2->GetPOV(0) == 0) ) {
			clawPick->Set( DoubleSolenoid::kReverse );
		}
		// On no POV pushed, stop the claw mid way up/down.
		else {
			clawPick->Set( DoubleSolenoid::kOff );
		}

		if( btnIntake->Get() || btnIntake2->Get() )
			mtrIntake->Set( 1.0 );
		else if ( btnOuttake->Get() || btnOuttake2->Get() )
			mtrIntake->Set( -1.0 );
		else
			mtrIntake->Set( 0.0 );




		SmartDashboard::PutNumber( "imuHeading", gyro->GetFusedHeading() );

		// On joystick trigger, reset the gyro to zero.
		if( joy->GetRawButton( 1 ) ) gyro->SetFusedHeading( 0.0, 0 );

		SmartDashboard::PutNumber( "chartOne", mtrLMaster->GetClosedLoopError(0) );
		SmartDashboard::PutNumber( "chartTwo", mtrLMaster->GetSelectedSensorVelocity(0) / 100.0 );
		SmartDashboard::PutNumber( "chartTwo", mtrLMaster->GetSelectedSensorPosition(0) / 100.0 );
	}

};


START_ROBOT_CLASS(Robot)
