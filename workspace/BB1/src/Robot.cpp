/*
 *	BB1 - Benzene Bots FRC 2018 Robot Source Code
 *
 *
 */
#include <iostream>
#include <string>

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Robot.h>
#include <CameraServer.h>
#include <Init.h>
#include <utils.h>
#include <buttons.h>
#include <elevator.h>
#include <climber.h>
#include <Autonomous.h>

TalonSRX *mtrLMaster, *mtrLSlave;
TalonSRX *mtrRMaster, *mtrRSlave;
TalonSRX *mtrElavator;
Victor *mtrIntake, *mtrClimber;

//CANifier *ultraSensor;

Joystick *joy, *joy2;
PigeonIMU *gyro;

Compressor *airCompressor;
Solenoid *clawClamp;
DoubleSolenoid *clawPick;

DigitalInput *elevatorSW;				// Home Position for Elevator

gains mpGains;							// PID Gains for Motion Profiling.

#include <Path_Finder.h>

Segment leftTraj[ 2048 ];				// Current trajectories are stored here.
Segment rightTraj[ 2048 ];

struct btns btns;						// Struct holds all values of the joysticks.
int cntDisable = 0;

#include <Motion_Profile.h>
#include <Arcade_Drive.h>
#include <sequencer.h>
#include <AutonChooser.h>
#include <intake.h>


class Robot : public TimedRobot {
private:
public:

	// ========================================================================
	void RobotInit() {
		mtrLMaster = new TalonSRX( 2 );				// Left Master
		mtrLSlave = new TalonSRX( 1 );				// Left Slave
		mtrRMaster = new TalonSRX( 4 );				// Right Master
		mtrRSlave = new TalonSRX( 3 );				// Right Slave
		gyro = new PigeonIMU( 0 );
		mtrElavator = new TalonSRX( 5 );
		elevatorSW = new DigitalInput( 0 );			// Home SW / RoboRIO Channel 0.
		mtrIntake = new Victor( 0 );
		mtrClimber = new Victor( 8 );
		airCompressor = new Compressor( 0 );
		joy = new Joystick( 0 );
		joy2 = new Joystick( 1 );
		clawClamp = new Solenoid( 0 );				// Cube clamping cylinders.
		clawPick = new DoubleSolenoid( 1, 2 );		// Claw raise / lower.
		//ultraSensor = new CANifier( 10 );


		// Initialize the four motors on the drivetrain.
		DrivetrainInit( mtrLMaster, mtrLSlave, mtrRMaster, mtrRSlave );

		InitElevator( mtrElavator );			// Init the elevator motor.
		AuxMotorInit( mtrClimber, mtrIntake );	// Init the Climber and Intake motors.
		GyroInit( gyro );						// Setup the Pigeon Gyro.
		JoystickInit( joy, joy2 );				// Joysticks Init

		// Setup all the pneumatics (Claw Clamp / Claw Pick).
		PnumaticsInit( airCompressor, clawPick, clawClamp );

		#ifdef PRACTICE_BOT
			//RebuildMotionProfiles();	// Note, this may take a bit of time.
		#endif

		// The mpThread handles shoveling trajectory points from the Top
		// API buffer out to the Talon(s).
		std::thread t1( mpThread );		// This starts the 100Hz thread right away.
    	t1.detach();					// Detach from this thread.

		Load_Waypoints();	// Call once to populate the waypoint data structures.

		AutonDashboardInit();

		IntakeInit( clawPick );

    	CameraServer::GetInstance()->StartAutomaticCapture();
	}

	// ========================================================================
	void RobotPeriodic() {

	}

	// ========================================================================
	void DisabledInit() {

		enXfer = false;

		mtrLMaster->NeutralOutput();
		mtrRMaster->NeutralOutput();

		// For the safety Mur-Dog, turn on them brakes!
		mtrLMaster->SetNeutralMode( NeutralMode::Brake );
		mtrRMaster->SetNeutralMode( NeutralMode::Brake );

		mtrLMaster->ClearMotionProfileTrajectories();
		mtrRMaster->ClearMotionProfileTrajectories();

		cntDisable = 0;
		printf( "Disabled...\n" );
	}

	// ========================================================================
	void DisabledPeriodic() {

		// After a couple seconds, switch back to coast mode so the robot is easy
		// to push around.
		if( ++cntDisable == 150 ) {
			mtrLMaster->SetNeutralMode( NeutralMode::Coast );
			mtrRMaster->SetNeutralMode( NeutralMode::Coast );
		}

		// Zeros the elevator position while disabled.
		resetElevatorPos( mtrElavator, elevatorSW );
	}

	// ========================================================================
	void AutonomousInit() {

		cntAuto = 0;

		AutoInit( mtrLMaster, mtrRMaster, gyro );


		std::string sGame = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		printf("Chosen Auton is %i \n", ChooseAuton( sGame ));

		AutonPathId pathIdx = ChooseAuton( sGame );

		//AutonPathId pathIdx = CenterRightSwitch;	// This line is for testing only.

    	seqInit( pathIdx );					// Init auto sequencer task.
		enAutoSeq( true );				// Start the sequencer.

		cntProfile = 0;					// Reset real-time task counter/timer.
		printf( "Auto Running...\n" );
	}

	// ========================================================================
	void AutonomousPeriodic() {
		// The sequencer thread takes care of running all the operations in Auto
		// mode.  The thread even prints the amount of total time taken to complete.
		//printf("Gyro Angle %f /n", gyro->GetFusedHeading());
	}

	// ========================================================================
	void TeleopInit() {
		printf( "TeleopInit...\n" );
		enXfer = false;
		DriverModeInit( airCompressor, mtrLMaster, mtrRMaster );
	}

	// ========================================================================
	void TeleopPeriodic() {
		OpRobot();	// This handles all the regular robot operations.
	}

	// ========================================================================
	void TestInit() {
		printf( "TestInit...\n" );
		DriverModeInit( airCompressor, mtrLMaster, mtrRMaster );
	}

	// ========================================================================
	void TestPeriodic() {
		//static uint32_t cnt=0;
		OpRobot();	// This handles all the regular robot operations.

		/*
		if( ++cnt > 10 ) {
			double duty[4];
			ultraSensor->GetPWMInput( CANifier::PWMChannel0, duty );
			printf( "Dist: %0.2f\n", duty[0] );
			cnt = 0;
			SmartDashboard::PutNumber( "chartOne", duty[0] );
			SmartDashboard::PutNumber( "chartTwo", 0 );
			SmartDashboard::PutNumber( "chartThree", 0 );
		}
		*/

		// Recalculate all the motion profile trajectories on a button press.
		if( btns.btn2[ eight2 ] )
			RebuildMotionProfiles();	// Note, this may take a bit of time.
	}

	// For teleop and test, this function does all the "regular" robot operating stuff.
	// ========================================================================
	void OpRobot() {
		double matchTime = DriverStation::GetInstance().GetMatchTime();
		frc::SmartDashboard::PutNumber("Match Time", matchTime);

		airCompressor->SetClosedLoopControl( COMPRESSOR );

		ReadButtons( &btns, joy, joy2 );	// Update all the buttons.
		ArcadeDrive( &btns );  				// Drive the robot drivetrain in arcade mode.

		// Handle all the claw and climber buttons here.
		ProcessClawButtons( &btns, clawPick, clawClamp, mtrIntake );
		//ProcessClimberButtons( &btns, mtrClimber );
		if(joy2->GetRawButton(12)){
			mtrClimber->Set(1.0);
		}else{
			mtrClimber->Set(0.0);
		}

		DriveElevator( joy2->GetRawAxis(1) * -1.0, mtrElavator, elevatorSW, &btns);

		// Update the smart dashboard.
		//UpdateSmartDash( gyro, mtrLMaster, mtrRMaster );
	}
};


START_ROBOT_CLASS(Robot)
