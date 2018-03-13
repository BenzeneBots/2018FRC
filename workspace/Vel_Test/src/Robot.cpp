//
//	Example code to test Velocity Contro, Motion Magic, and PathFinder.
//

#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "Robot.h"
#include "pathfinder.h"

#define MAX_SP		480.0	// Max Measured RPM (138.2 ft/s with 6" wheels = 42 m/s).
#define Fgain		0.3119	// 1023 Max Velocity / 3280nu(Max Native Units / 100ms measured).

Segment *trajectory;
Segment *leftTrajectory;
Segment *rightTrajectory;
int trajLen = 0;

// Given a small set of waypoints, calculate a bunch of trajectory points that
// make a smooth curved path for each side of the drivetrain.
// ============================================================================
void PathFinder() {
	int POINT_LENGTH = 3;

	Waypoint points[POINT_LENGTH];

	Waypoint p1 = { -4, -1, d2r(45) };      // Waypoint @ x=-4, y=-1, exit angle=45 degrees
	Waypoint p2 = { -1, 2, 0 };             // Waypoint @ x=-1, y= 2, exit angle= 0 radians
	Waypoint p3 = {  2, 4, 0 };             // Waypoint @ x= 2, y= 4, exit angle= 0 radians
	points[0] = p1;
	points[1] = p2;
	points[2] = p3;

	TrajectoryCandidate candidate;

	// Prepare the Trajectory for Generation.
	//
	// Arguments:
	// Fit Function:        FIT_HERMITE_CUBIC or FIT_HERMITE_QUINTIC
	// Sample Count:        PATHFINDER_SAMPLES_HIGH (100 000)
	//                      PATHFINDER_SAMPLES_LOW  (10 000)
	//                      PATHFINDER_SAMPLES_FAST (1 000)
	// Time Step:           0.001 Seconds
	// Max Velocity:        15 m
	// Max Acceleration:    10 m/s/s
	// Max Jerk:            60 m/s/s/s
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, 0.001, 15.0, 10.0, 60.0, &candidate);

	trajLen = candidate.length;

	// Array of Segments (the trajectory points) to store the trajectory in
	trajectory = (Segment*)malloc( trajLen * sizeof(Segment) );

	// Generate the trajectory
	int result = pathfinder_generate(&candidate, trajectory);
	if (result < 0) {
	    // An error occured
	    printf( "Uh-Oh! Trajectory could not be generated!\n" );
	}
	else {
		printf( "Trajectory Length: %d\n", trajLen );
	}

	double tm=0;
	for (int i = 0; i < trajLen; i++) {
	    Segment s = trajectory[i];
	    printf( "Time: %f\n", tm += s.dt );
	    printf( "Time Step: %f\n", s.dt );
	    printf( "Coords: (%f, %f)\n", s.x, s.y );
	    printf( "Position (Distance): %f\n", s.position );
	    printf( "Velocity: %f\n", s.velocity );
	    printf( "Acceleration: %f\n", s.acceleration );
	    printf( "Jerk (Acceleration per Second): %f\n", s.jerk );
	    printf( "Heading (radians): %f\n\n", s.heading );
	}

	leftTrajectory = (Segment*)malloc( trajLen * sizeof(Segment) );
	rightTrajectory = (Segment*)malloc( trajLen * sizeof(Segment) );

	// The distance between the left and right sides of the wheelbase is 0.6m.
	double wheelbase_width = 0.6;

	// Generate the Left and Right trajectories of the wheelbase using the
	// originally generated trajectory.
	pathfinder_modify_tank( trajectory, trajLen, leftTrajectory, rightTrajectory, wheelbase_width );

	// Free the memory used by "trajectory" which was the center path of trajectory
	// points.  The trajectory points in leftTrajectory and rightTrajectory should
	// be used to drive the robot during auto.
	free( trajectory );
}



class Robot: public IterativeRobot {
private:
	TalonSRX * mtrLeftMaster = new TalonSRX( 2 );
	TalonSRX * mtrLSlave = new TalonSRX( 1 );
	Joystick * _joy = new Joystick(1);
	std::string _sb;
	int cnt = 0;

	void RobotInit() {
        /* first choose the sensor */
		mtrLeftMaster->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		mtrLeftMaster->SetSensorPhase(true);

		/* set the peak and nominal outputs */
		mtrLeftMaster->ConfigNominalOutputForward(0, kTimeoutMs);
		mtrLeftMaster->ConfigNominalOutputReverse(0, kTimeoutMs);
		mtrLeftMaster->ConfigPeakOutputForward(1, kTimeoutMs);
		mtrLeftMaster->ConfigPeakOutputReverse(-1, kTimeoutMs);
		/* set closed loop gains in slot0 */
		mtrLeftMaster->Config_kF(kPIDLoopIdx, Fgain, kTimeoutMs);
		mtrLeftMaster->Config_kP(kPIDLoopIdx, 0.3,   kTimeoutMs);	// (10% * 1023) / (350nu worst err measured).
		mtrLeftMaster->Config_kI(kPIDLoopIdx, 0.003, kTimeoutMs);	// Default: kP / 100
		mtrLeftMaster->Config_kD(kPIDLoopIdx, 3.0,   kTimeoutMs);	// Default: kP * 10

		mtrLSlave->Set( ControlMode::Follower, mtrLeftMaster->GetDeviceID() );

		mtrLeftMaster->SetStatusFramePeriod( StatusFrameEnhanced::Status_13_Base_PIDF0, 10, kTimeoutMs );
		mtrLeftMaster->SetStatusFramePeriod( StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs );

		mtrLeftMaster->ConfigMotionCruiseVelocity( 1000, kTimeoutMs );
		mtrLeftMaster->ConfigMotionAcceleration( 500, kTimeoutMs );
		mtrLeftMaster->SetSelectedSensorPosition( 0, 0, kTimeoutMs );

		PathFinder();
	}

	void TeleopInit() {
		mtrLeftMaster->SetSelectedSensorPosition( 0, 0, 0);

	}

	/**
	 * This function is called periodically during operator control
	 */
	void TeleopPeriodic() {
		/* get gamepad axis */
		double leftYstick = -1.0 * _joy->GetY();
		double motorOutput = mtrLeftMaster->GetMotorOutputPercent();
		/* prepare line to print */
		_sb.append("\t    out:");
		_sb.append(std::to_string(motorOutput));
		_sb.append("\t   spd:");
		float fVel = mtrLeftMaster->GetSelectedSensorVelocity(kPIDLoopIdx);
		_sb.append(std::to_string( fVel ));

		// While button1 "trigger" is held down, closed-loop on target velocity.
		if (_joy->GetRawButton(1)) {
        	/* Speed mode */
			/* Convert 400 RPM to units / 100ms.
			 * 4096 Units/Rev * 400 RPM / 600 100ms/min in either direction:
			 * velocity setpoint is in units/100ms
			 */
			double targetVelocity_UnitsPer100ms = leftYstick * 400.0 * 4096 / 600;
        	mtrLeftMaster->Set(ControlMode::Velocity, targetVelocity_UnitsPer100ms);

			/* append more signals to print when in speed mode. */
			_sb.append("\t    errNative:");
			float fErr = mtrLeftMaster->GetClosedLoopError(kPIDLoopIdx);
			_sb.append(std::to_string(fErr));
			_sb.append("\t    SetPt:");
			_sb.append(std::to_string(targetVelocity_UnitsPer100ms));

			SmartDashboard::PutNumber( "chartOne", fErr );
			SmartDashboard::PutNumber( "chartTwo", targetVelocity_UnitsPer100ms );
			SmartDashboard::PutNumber( "chartThree", fVel );
		}

		// Else, if "thumb" button is held down, use Motion Magic.
		else if(_joy->GetRawButton(2)) {
			double curDist, dist = 10.0 * 4096;	// Ten wheel rotations.

			//MotionProfileStatus *st = new MotionProfileStatus;

			curDist = mtrLeftMaster->GetSelectedSensorPosition(0);
			/*
			mtrLeftMaster->GetMotionProfileStatus( *st );
			if( st->isLast ) {
				printf( "Magic Done\n" );
			}
			*/
			mtrLeftMaster->Set(ControlMode::MotionMagic, dist );

			SmartDashboard::PutNumber( "chartOne", curDist );
			SmartDashboard::PutNumber( "chartTwo", mtrLeftMaster->GetSelectedSensorVelocity(0) * 10 );
			//SmartDashboard::PutNumber( "chartThree", fVel );
        }

		// Else, neither button is held down.
		else {
			/* Percent voltage mode */
			mtrLeftMaster->Set(ControlMode::PercentOutput, leftYstick);
			mtrLeftMaster->Set( ControlMode::MotionMagicArc, 10.0 );
		}


		// Print every ten loops.
		if (++cnt >= 10) {
			cnt = 0;
			printf("%s\n",_sb.c_str());
		}
		_sb.clear();
	}
};

START_ROBOT_CLASS(Robot)
