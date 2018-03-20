/*
 *	BB1 - Benzene Bots FRC 2018 Robot Source Code
 *
 *
 */
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Robot.h>
#include <CameraServer.h>

// Defining "DEBUG" enables extra debug stuff. Like smart dashboard data.
// Comment out this line during competition.
#define DEBUG

PigeonIMU *imu;
TalonSRX *mtrLMaster, *mtrLSlave;
TalonSRX *mtrRMaster, *mtrRSlave;
TalonSRX *mtrElavator;
Victor *mtrIntake;

Joystick *joy;
PigeonIMU *gyro;
JoystickButton *btnStrait;
JoystickButton *btnDrive;

JoystickButton *btnIntake;
JoystickButton *btnOuttake;
JoystickButton *btnClawClose;
JoystickButton *btnClawOpen;
JoystickButton *btnCalcPaths;	// Used in Test to recalculate all the Motion Paths.

Compressor *airCompressor;
Solenoid *clawClamp;
DoubleSolenoid *clawPick;

#include <Path_Finder.h>

Segment leftTraj[ 2048 ];
Segment rightTraj[ 2048 ];

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
		mtrElavator = new TalonSRX( 5 );
		mtrIntake = new Victor( 0 );
		joy = new Joystick( 1 );
    	btnStrait = new JoystickButton( joy, 2 );	// Thumb Button.
    	btnIntake = new JoystickButton( joy, 1 );	// Trigger Button.
    	btnOuttake = new JoystickButton( joy, 7 );
    	btnClawOpen = new JoystickButton( joy, 3 );
    	btnClawClose = new JoystickButton( joy, 4 );
    	btnCalcPaths = new JoystickButton( joy, 11 );	// In Test to recalc paths.

		gyro = new PigeonIMU( 0 );

		airCompressor = new Compressor(0);
		clawClamp = new Solenoid( 0 );
		clawPick = new DoubleSolenoid( 1, 2 );

		clawPick->Set( DoubleSolenoid::kOff );
		clawClamp->Set( false );

		mtrIntake->Set( 0.0 );

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
		mtrLMaster->ConfigNominalOutputForward(0, kTO);
		mtrLMaster->ConfigNominalOutputReverse(0, kTO);
		mtrLMaster->Config_kF(kPIDLoopIdx, mpGains.ff,	kTO);
		mtrLMaster->Config_kP(kPIDLoopIdx, mpGains.p,   kTO);
		mtrLMaster->Config_kI(kPIDLoopIdx, mpGains.i,   kTO);
		mtrLMaster->Config_kD(kPIDLoopIdx, mpGains.d,   kTO);
		mtrLMaster->Config_IntegralZone( 0, mpGains.iZone, kTO );
		mtrLMaster->ConfigPeakOutputForward( mpGains.peakOut, kTO );
		mtrLMaster->ConfigPeakOutputReverse(-1 * mpGains.peakOut, kTO);

		// Right Side
		mtrRMaster->ConfigNominalOutputForward(0, kTimeoutMs);
		mtrRMaster->ConfigNominalOutputReverse(0, kTimeoutMs);
		mtrRMaster->Config_kF(kPIDLoopIdx, mpGains.ff,	kTO);
		mtrRMaster->Config_kP(kPIDLoopIdx, mpGains.p,   kTO);
		mtrRMaster->Config_kI(kPIDLoopIdx, mpGains.i,   kTO);
		mtrRMaster->Config_kD(kPIDLoopIdx, mpGains.d,   kTO);
		mtrRMaster->Config_IntegralZone( 0, mpGains.iZone, kTO );
		mtrRMaster->ConfigPeakOutputForward( mpGains.peakOut, kTO );
		mtrRMaster->ConfigPeakOutputReverse(-1 * mpGains.peakOut, kTO);

		// Setup Follower Slaves.
		mtrLSlave->Set( ControlMode::Follower, mtrLMaster->GetDeviceID() );
		mtrRSlave->Set( ControlMode::Follower, mtrRMaster->GetDeviceID() );

		// Profile uses 10 ms timing.
		mtrLMaster->ConfigMotionProfileTrajectoryPeriod( 10, kTimeoutMs );
		mtrRMaster->ConfigMotionProfileTrajectoryPeriod( 10, kTimeoutMs );
		// Status 10 provides the trajectory target for motion profile AND motion magic.
		mtrLMaster->SetStatusFramePeriod(
				StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);
		mtrRMaster->SetStatusFramePeriod(
				StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);

		mtrLMaster->ClearMotionProfileTrajectories();
		mtrRMaster->ClearMotionProfileTrajectories();

		TrajectoryDuration dt = TrajectoryDuration_10ms;
		mtrLMaster->ConfigMotionProfileTrajectoryPeriod( dt, kTimeoutMs);
		mtrRMaster->ConfigMotionProfileTrajectoryPeriod( dt, kTimeoutMs);

		mtrLMaster->ChangeMotionControlFramePeriod( TrajectoryDuration_10ms / 2 );
		mtrRMaster->ChangeMotionControlFramePeriod( TrajectoryDuration_10ms / 2 );

		gyro->SetFusedHeading( 0.0, 0 );

		// The mpThread handles shoveling trajectory points from the Top
		// API buffer out to the Talon(s).
		std::thread t1(mpThread);		// This starts the 100Hz thread right away.
    	t1.detach();					// Detach from this thread.

		Load_Waypoints();	// Call once to populate the waypoint data structures.

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
		//char s[120];
		printf( "AutoInit...\n" );

		mtrLMaster->NeutralOutput();
		mtrRMaster->NeutralOutput();

		mtrLMaster->SetSelectedSensorPosition( 0, 0, 0 );
		mtrRMaster->SetSelectedSensorPosition( 0, 0, 0 );
		gyro->SetFusedHeading( 0.0, 0 );

		// Load the named profile from the file-system into the RoboRIO API buffer.
		LoadProfile( Mid_SwitchRight, false );

		mtrLMaster->SetIntegralAccumulator( 0.0, 0, 0 );
		mtrRMaster->SetIntegralAccumulator( 0.0, 0, 0 );

		cntProfile = 0;		// Reset real-time task counter/timer.
		printf( "Auto Pts Running...\n" );
	}

	// ========================================================================
	void AutonomousPeriodic() {
		static uint16_t cnt=0, seq = 0;
		bool flgMoving;

		if( seq == 0 ) {
			flgMoving = RunProfile();	// Run the loaded profile.
			if( flgMoving == false ) {
				seq += 1;
				printf( "Seg 0 Done\n" );
			}
		}
		if( seq == 1 ) {
			LoadProfile( Mid_SwitchRight, true );
			seq += 1;
			printf( "Seg 1 Done\n" );
		}
		if( seq == 2 ) {
			flgMoving = RunProfile();
			if( flgMoving == false ) {
				seq += 1;
				printf( "Seg 2 Done\n" );
			}
		}

		cnt += 1;	// Static var used for timing events.

		#ifdef DEBUG	// Turn debug network traffic off at competition.
		SmartDashboard::PutNumber( "imuHeading", gyro->GetFusedHeading() );
		SmartDashboard::PutNumber( "chartOne", mtrLMaster->GetClosedLoopError(0) );
		SmartDashboard::PutNumber( "chartTwo", mtrLMaster->GetSelectedSensorVelocity(0) );
		SmartDashboard::PutNumber( "chartThree", mtrLMaster->GetSelectedSensorPosition(0) );
		#endif
	}

	// ========================================================================
	void TeleopInit() {
		printf( "TeleopInit...\n" );
		mtrLMaster->SetSelectedSensorPosition( 0, 0, 0 );
		mtrLMaster->SetSelectedSensorPosition( 0, 0, 0 );
		airCompressor->SetClosedLoopControl( true);

		mtrLMaster->NeutralOutput();
		mtrRMaster->NeutralOutput();

		mtrLMaster->ConfigOpenloopRamp( 0.1, kTO );
		mtrRMaster->ConfigOpenloopRamp( 0.1, kTO );

		SmartDashboard::PutNumber( "chartOne", mtrLMaster->GetSelectedSensorVelocity(0) );
		SmartDashboard::PutNumber( "chartTwo", 0.0 );
		SmartDashboard::PutNumber( "chartThree", 0.0 );		
	}

	// ========================================================================
	void TeleopPeriodic() {
		//static uint16_t cnt=0;

		airCompressor->SetClosedLoopControl( true);

		ArcadeDrive();

		if( btnClawOpen->Get() ) {
			clawClamp->Set( true );
		}
		if( btnClawClose->Get() ) {
			clawClamp->Set( false );
		}

		// Push POV up to raise claw.
		if( (joy->GetPOV(0) == 180)  ) {
			clawPick->Set( DoubleSolenoid::kForward );
		}
		// Push POV down to lower claw.
		else if( (joy->GetPOV(0) == 0) ) {
			clawPick->Set( DoubleSolenoid::kReverse );
		}
		// On no POV pushed, stop the claw mid way up/down.
		else {
			clawPick->Set( DoubleSolenoid::kOff );
		}

		if( btnIntake->Get() )
			mtrIntake->Set( (-1.0 * joy->GetThrottle() + 1.0) / 2.0 );
		else if ( btnOuttake->Get() )
			mtrIntake->Set( -1.0 );
		else
			mtrIntake->Set( 0.0 );

		// On joystick trigger, reset the gyro to zero.
		if( joy->GetRawButton( 1 ) ) gyro->SetFusedHeading( 0.0, 0 );

		SmartDashboard::PutNumber( "chartOne", gyro->GetFusedHeading() );
		SmartDashboard::PutNumber( "chartTwo", mtrLMaster->GetSelectedSensorVelocity( 0 ) );
		SmartDashboard::PutNumber( "chartThree", mtrLMaster->GetSelectedSensorPosition( 0 ) );
		SmartDashboard::PutNumber( "imuHeading", gyro->GetFusedHeading() );
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

		// On button 11 press, recalculate all the motion profile trajectories.
		// Note, this may take a bit of time.
		if( btnCalcPaths->Get() ) {
			printf( "Recalculating Motion Trajectories...\n" );

			Load_Waypoints();	// Load all the data into the waypoint structures.

			// Step thru and calculate each path.  The result is stored as a 
			// binary/CSV file on the RoboRIO file-system.
			for ( int idx=0 ; idx < NUM_PATHS ; idx++ ) {
				printf( "Calculating PathFinder Path: %d\n", idx );
				PathFinder( idx );
			}
			printf( "Info: PathFinder Done\n" );
		}
	}

};


START_ROBOT_CLASS(Robot)
