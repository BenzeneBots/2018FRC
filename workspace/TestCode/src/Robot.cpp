// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//	Robot Test Code:									Started By: Jim Kemp
//																	11/26/2017
//
//
//	This is a scratch pad area to try out new hardware and software.
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include <iostream>
#include <memory>
#include <string>

#include <Thread>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "NetworkTables/NetworkTable.h"

#include "WPILib.h"
#include "PigeonImu.h"

#include "GenericHID.h"

#define PID_RATE	0.05	// This is the update rate (in seconds) for the drivetrain.

Joystick *Joy1;
JoystickButton *btnA, *btn4, *btn5, *btn6;
JoystickButton *btnX, *btnB;
Talon *leftDrv, *rightDrv;
DoubleSolenoid *shifter;
Solenoid *gearOpener;
PowerDistributionPanel *pdp;
Encoder *leftEnc;
Encoder *rightEnc;
//PIDController *pidLeft, *pidRight;
PigeonImu *gyro;
Talon *leftMtr, *rightMtr;
Timer *tm;
DigitalOutput *dioOut1;
CANTalon *shooterMotor;
Talon *murWheel1, *murWheel2;
GearTooth *leftMtrSp, *rightMtrSp;
std::shared_ptr<NetworkTable> table;

// Speed and Direction
double spCmd = 0.0, angCmd = 0.0;
uint8_t rum=0;

void thread200Hz( void );	// This function is setup as the realtime control loop.

// ============================================================================
// PWM Channels
// 0		Right Drive #1
// 1		Floor #1
// 2		Intake #1
// 3		Left Drive #1 & #2
// 4		Floor #2
// 5		Climber
//
// Digital IO Channels
// 0 & 1	Left Encoder (360 Cnts / Wheel Rev.)
// 2 & 3	Right Encoder (250 Cnts / Wheel Rev.)
// 4		Digital output pin.
//
// ============================================================================
class Robot: public frc::IterativeRobot {
public:

	// ------------------------------------------------------------------------
	void RobotInit() {
		// The chooser only works with the Java Dashboard!  The NI dashboard
		// uses a simple "getstring" instead.
		//chooser.AddDefault(autoNameDefault, autoNameDefault);
		//chooser.AddObject(autoNameCustom, autoNameCustom);
		//frc::SmartDashboard::PutData("Auto Modes", &chooser);

		// Setup Gyro
		gyro = new PigeonImu( 14 );
		gyro->SetFusedHeading( 0.0 );

		// Setup Motor Controllers
		leftDrv = new Talon( 3 );	leftDrv->SetInverted( false );
		rightDrv = new Talon( 0 );	rightDrv->SetInverted( true );

		// Setup Encoders
		leftEnc = new Encoder( 0, 1, true, leftEnc->k4X );
		leftEnc->SetDistancePerPulse( 1.0 / 360.0 );
		leftEnc->SetPIDSourceType( PIDSourceType::kRate );

		rightEnc = new Encoder( 2, 3, false, rightEnc->k4X );
		rightEnc->SetDistancePerPulse( 1.0 / 250.0 );
		rightEnc->SetPIDSourceType( PIDSourceType::kRate );

		// PID Control for left and right drivetrain control.
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		//**pidLeft = new PIDController(  0.1, 0.01, 0.0, 1.0, leftEnc,  leftDrv,  PID_RATE );
		//**pidRight = new PIDController( 0.1, 0.01, 0.0, 1.0, rightEnc, rightDrv, PID_RATE );
		//**pidLeft->SetInputRange( -8.0, 8.0 );
		//**pidRight->SetInputRange( -8.0, 8.0 );
		//**pidLeft->SetTolerance( 15.0 );
		//**pidRight->SetTolerance( 15.0 );
		//pidLeft->SetOutputRange( 0.0, 1.0 );
		//pidRight->SetOutputRange( 0.0, 1.0 );

		// Setup Everything Else
		gearOpener = new Solenoid( 0 );				// Channel 0
		shifter = new DoubleSolenoid( 1, 2 );		// Setup double acting sol on channel 1, 2.

		//murWheel1 = new Talon( 4 );
		// murWheel2 = new Talon( ??? );		// ToDo Assign Channel

		// Shooter - CAN TalonSRX Setup
		shooterMotor = new CANTalon( 11 );	// CAN Id = 11
		shooterMotor->SetInverted( true );

		// Shooter - Encoder Setup
		shooterMotor->SetFeedbackDevice( CANTalon::CtreMagEncoder_Relative );
		shooterMotor->SetSensorDirection( true );
		shooterMotor->ConfigEncoderCodesPerRev( 4096 );

		// Shooter - PID Controller Setup
		shooterMotor->SetTalonControlMode( CANTalon::kDisabled );
		// Use slot one (can be zero or one).
		shooterMotor->SelectProfileSlot( 1 );											
		shooterMotor->ConfigNominalOutputVoltage( +0.f, -0.f );
		shooterMotor->ConfigPeakOutputVoltage( +12.f, -12.f );
		// Disable Limit SW and Soft Limits.
		shooterMotor->ConfigLimitMode( CANTalon::kLimitMode_SrxDisableSwitchInputs );	
		// Ramp: (Xsec/V / 12V)
		shooterMotor->SetVoltageRampRate( 12.0 );										

		// Murali - Two Wheel Shooter
		leftMtr = new Talon( 4 );
		leftMtr->SetInverted( false );
		leftMtrSp = new GearTooth( 4 );

		rightMtr = new Talon( 5 );
		rightMtr->SetInverted( true );
		rightMtrSp = new GearTooth( 5 );

		pdp = new PowerDistributionPanel;

		Joy1 = new Joystick( 1 );
		btnA = new JoystickButton( Joy1, 1 );		// "A" Button Gear Open (while held).
		btnB = new JoystickButton( Joy1, 2 );		// "B" Button Shiter to High Gear
		btnX = new JoystickButton( Joy1, 3 );		// "X" Button Shiter to Low Gear
		btn4 = new JoystickButton( Joy1, 4 );		// "Y" Button Shoot!!!
		btn5 = new JoystickButton( Joy1, 5 );		// "Left Trigger" Ball Intake
		btn6 = new JoystickButton( Joy1, 6 );		// "Right Trigger" Ball Intake Reverse

		dioOut1 = new DigitalOutput( 4 );	// Setup test output pin.

		table = NetworkTable::GetTable( "DriverStation" );

    	std::thread t1(thread200Hz);	// This starts the thread right away!
    	t1.detach();					// Detach from this thread.
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	// ------------------------------------------------------------------------
	void AutonomousInit() override {
		//autoSelected = chooser.GetSelected();
		std::string autoSelected = 
			SmartDashboard::GetString( "Auto Selector", autoNameDefault );
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if ( autoSelected == autoNameCustom ) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	// ------------------------------------------------------------------------
	void AutonomousPeriodic() {
		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	// ------------------------------------------------------------------------
	void DisabledInit() override {
		//Scheduler::GetInstance()->Run();
		printf( "Disabled\n" );
		shifter->Set( shifter->kOff );

		// Make sure motors are stopped.
		leftDrv->Set( 0.0 );
		rightDrv->Set( 0.0 );

		Joy1->SetRumble( frc::GenericHID::kLeftRumble, 0.0 );
		Joy1->SetRumble( frc::GenericHID::kRightRumble, 0.0 );
		rum = 0;
	}

	// ------------------------------------------------------------------------
	void TeleopInit() {

	}

	// ------------------------------------------------------------------------
	void TeleopPeriodic() {

	}

	// ------------------------------------------------------------------------
	void TestInit() override {
		printf( "TestInit...\n" );
		shifter->Set( shifter->kForward );	// Set solenoid to a default state.
		gyro->SetFusedHeading( 0.0 );		// This just zeros the heading.

		//pidLeft->Enable();
		//pidRight->Enable();

		leftEnc->Reset();		// Zero encoder distance.
		rightEnc->Reset();

		leftMtr->Set( 0.0 );	// Init motors.
		rightMtr->Set( 0.0 );

		//Joy1->SetRumble( frc::GenericHID::kLeftRumble, 0.9 );
		Joy1->SetRumble( frc::GenericHID::kRightRumble, 0.9 );
		rum = 0;
	}

	// ------------------------------------------------------------------------
	void TestPeriodic() {
		//double angle, rate;		// Gyro values.
		//double joyX, joyY;			// Joystick value.
		static uint16_t cntDebug=0;
		static bool flg=false;

		lw->Run();

		// Run cntDebug up to 50 and then roll over back to zero.  When cntDebug
		// is zero then print out debug messages.  This slows the rate of messages!
		cntDebug = (cntDebug < 50) ? (cntDebug + 1) : 0;
/*
		spCmd = Joy1->GetY( ) * -1.0;	// Joystick Speed Axis is inverted!
		angCmd = Joy1->GetRawAxis( 4 );	// Joystick Direction

		updateDrive();	// Drive the robot based on the joystick values.

		gyroTestForReset();				// Reset gyro if SmartDash button is pressed.
		gyroUpdate( &angle, &rate );	// Update angle / rate values + SmartDash.

		gearOpener->Set( btn1->Get() );			// Open Gear Slot (while button is held).
		shootBalls( btn4->Get() );				// Shoot balls (while button is held).
		ballIntake( btn5->Get(), btn6->Get() );	// Intake balls on button press.
		runClimber( Joy1->GetRawAxis( 3 ) );

		runMurMotors( Joy1->GetRawAxis( 2 ) );	// Left Throttle

		// Switch between High/Low gear on the shifter based on buttons.
		//if( btn2->Get() )		shifter->Set( shifter->kForward );
		//else if( btn3->Get() )	shifter->Set( shifter->kReverse );
		*/

		if( rum < 200 ) {
			rum += 1;
		}
		else if( rum == 200 ) {
			Joy1->SetRumble( frc::GenericHID::kLeftRumble, 0.0 );
			Joy1->SetRumble( frc::GenericHID::kRightRumble, 0.0 );
			printf( "Rumble Over\n" );
			rum = 201;
		}


		if( btnX->Get() ) {
			shooterMotor->Set( 0.5 );
		}
		if( btnA->Get() ) {
			shooterMotor->Set( 0.75 );
		}
		if( btnB->Get() ) {
			shooterMotor->Set( 0.0 );
		}

		double leftMtrFb = leftMtrSp->GetPeriod();
		if( leftMtrFb > 0.006 ) {
			leftMtrFb = (1.0 / leftMtrFb) * 60.0;
			// Overwrite Normal Value
			SmartDashboard::PutNumber( "leftEncoder", leftMtrFb );		
		}

		double rightMtrFb = rightMtrSp->GetPeriod();
		if( rightMtrFb > 0.006 ) {
			rightMtrFb = (1.0 / rightMtrFb) * 60.0;
			// Overwrite Normal Value
			SmartDashboard::PutNumber( "rightEncoder", rightMtrFb );		
		}
		//if( ! cntDebug ) printf( "Rate: %d   %d\n", (int)leftMtrFb, (int)rightMtrFb );

		/*
		double m1Curr = pdp->GetCurrent( 7 );		// M1 uses pdp fuse channel #7.
		SmartDashboard::PutNumber( "m1Current", m1Curr );

		//double l = leftEnc->GetDistance();
		double r = rightEnc->GetDistance();
		double lRate = leftEnc->GetRate();
		double rRate = rightEnc->GetRate();
		double wSp = wheelSpeed->GetPeriod();

		if( ! cntDebug ) 
			printf( "Enc: %0.2f     %0.2f     %0.2f %f\n", lRate, rRate, angle, wSp );



		 *
		double per = murSpeed1->GetPeriod();
		if( per > 0.006 ) {
			double shRate = (1.0 / per ) * 60.0;
			if( ! cntDebug ) printf( "Rate: %f\n", shRate );
			SmartDashboard::PutNumber( "leftEncoder", shRate );		// Overwrite Normal Value
		}

		*/
		int shRate = shooterMotor->GetSpeed();
		SmartDashboard::PutNumber( "leftEncoder", shRate );
		//SmartDashboard::PutNumber( "leftEncoder", l );
		//SmartDashboard::PutNumber( "rightEncoder",r * -1.0 );
		//SmartDashboard::PutNumber( "imuHeading", angle );


		//if( ! cntDebug ) printf( "Gyro: %0.2f\n", angle );


		if( cntDebug == 0 ) {
			table->PutBoolean( "LED1", flg );
			flg = flg ? false : true;
		}
	}

// ============================================================================
// ============================================================================
private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;

	// ------------------------------------------------------------------------
	void runMurMotors( double sp ) {

		murWheel1->Set( sp );
	}


	// ------------------------------------------------------------------------
	void runClimber( double sp ) {
		#ifdef DEMO_BOT
			climberMotor->Set( sp );
		#endif
	}

	// ------------------------------------------------------------------------
	void ballIntake( bool btnFwd, bool btnRev ) {
		#ifdef DEMO_BOT
			if( btnFwd ) {
				intakeMotor->Set( 0.75 );	// Run motor.
			}
			else if( btnRev ) {
				intakeMotor->Set( -0.75 );	// Run motor in reverse.
			}
			else {
				intakeMotor->Set( 0.0 );	// Stop motor.
			}
		#endif
	}

	// While the flag is true, shoot balls using the shooter and live floor.
	// ------------------------------------------------------------------------
	void shootBalls( bool btnFlg ) {
		//static float tmStart=0.0;

		// Shoot balls on flag true.
		if( btnFlg ) {
			shooterMotor->Set( 0.5 );
			// Start shooter intake x seconds after shooter spins up.
			//if( tm->GetFPGATimestamp() > (tmStart + 1.5) )
			//	shooterIntakeMotor->Set( 0.4 );
		}
		else {
			shooterMotor->Set( 0.0 );			// Stop motors if flag is false.
			//shooterIntakeMotor->Set( 0.0 );
			//tmStart = tm->GetFPGATimestamp();	// Keep start time reset to current time.
		}
	}

	// Update the drive motors using joystick control.
	// ------------------------------------------------------------------------
	void updateDrive( void ) {


		/*
		double fwd = Joy1->GetRawAxis( 4 ) * -1.0;
		double ang = Joy1->GetY();

		leftDrv->Set( (fwd + ang) / 2.0 );
		leftDrv2->Set( (fwd + ang) / 2.0 );

		rightDrv->Set( (fwd - ang) / 2.0 );
		rightDrv2->Set( (fwd - ang) / 2.0 );
		*/
	}

	// Set Motor1 speed.
	// ------------------------------------------------------------------------
	void SetM1( double sp, double scale ) {
		//m1->Set( sp / scale );
	}

	// Based on SmartDash Reset button state, Reset / Calibrate Gyro.
	// ------------------------------------------------------------------------
	void gyroTestForReset() {

		// Test if reset button on the SmartDash is pressed.  Assume no reset request.
		bool flg = SmartDashboard::GetBoolean( "imuReset", false );

		if( flg ) {
			gyro->SetFusedHeading( 0.0 );
			// Reset the SmartDash button back to off / false.
			SmartDashboard::PutBoolean( "imuReset", false );
		}

	}

	// Update and return the gyro values.  Also, update the SmartDash values.
	// ------------------------------------------------------------------------
	void gyroUpdate( double *angle, double *rate ) {

		double rates[3];	// Holes gyro rates for x, y, and z.
		gyro->GetRawGyro( rates );
		*rate = rates[2];	// Return gyro rate for Z axis.
		*angle = gyro->GetFusedHeading();

		SmartDashboard::PutNumber( "imuHeading", *angle );
		SmartDashboard::PutNumber( "imuRate", *rate );
	}
};

START_ROBOT_CLASS(Robot)

#define DEADBAND	0.010
#define SP_ACCEL	0.005	// Speed percent speed change every 5ms.
#define SP_DECEL	0.100
#define DR_ACCEL	0.003	// Direction percent change every 5ms.
#define DR_DECEL	0.100


// This thread is designed to run at 200Hz (or every 5ms).
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void thread200Hz( void ) {
	//double tmNow=0, tmOld=0;
	tm = new Timer();
	static uint16_t cnt=1;		// Don't really need static here since this never returns.
	static bool flg=false;
	static double sp=0.0, dr=0.0;
	double acl=0.01;

	while( 1 ) {

		if( frc::RobotState::IsEnabled() ) {
			dioOut1->Set( flg = !flg );			// Toggle the test IO pin.

			// Limit the rate at which the speed can be changed.
			if( fabs(spCmd) > DEADBAND ) {	// Check if joystick is outside the deadband.

				// Figure out if accelerating or decelerating.
				if( fabs(spCmd) > fabs(sp) ) 	acl = SP_ACCEL;		// Set Accel Rate
				else							acl = SP_DECEL;		// Set Decel Rate

				// Bump current speed by accel rate.
				if( sp < spCmd ) sp += acl;
				if( sp > spCmd ) sp -= acl;
			}
			else {
				// Else, must be inside the deadband so reset the speed.
				sp = 0.0;
			}

			// Limit the rate of change on the steering too.  Same logic as above for speed.
			if( fabs(angCmd) > 0.01 ) {
				if( fabs(angCmd) > fabs(dr) ) 	acl = DR_ACCEL;		// Accel Rate
				else							acl = DR_DECEL;		// Decel Rate

				if( dr < angCmd ) dr += acl;
				if( dr > angCmd ) dr -= acl;
			}
			else {
				dr = 0.0;
			}

			leftDrv->Set( fmin((sp + dr), 1.0) );
			rightDrv->Set( fmax((sp - dr), -1.0) );
		}
		else {
			// Else, if not enabled then reset everything.
			leftDrv->Set( 0.0 );
			rightDrv->Set( 0.0 );
			sp = 0.0;
			dr = 0.0;
		}

		// Every 200th loop (or once a second), print out some stats.
		//tmNow = tm->GetFPGATimestamp();		// First thing, get the timestamp saved!
		if( ++cnt >= 200 ) {
			//printf( "Time: %f\n", tmNow - tmOld );
			//tmOld = tmNow;
			cnt = 1;
		}
		//tmOld = tmNow;

		// Sleep the thread for 5 milliseconds.  Note, there is about 90uS of overhead.
		std::this_thread::sleep_for( std::chrono::microseconds( 5000 - 90 ) );
	}
}

