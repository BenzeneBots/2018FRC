/*
 * Drive.cpp
 *
 *  Created on: Jan 28, 2018
 *      Author: Murali
 */

#include <Subsystems/Drive.h>
#include <ctre/Phoenix.h>
#include <WPILib.h>
#include <Math.h>
#include <pathfinder.h>


//Defined drive constants
#define INCHES_PER_TICK 0.004636 //measured 02-20-18
#define TICKS_PER_INCH 217.299549
#define TICKS_PER_DEGREE 1600/360
#define RESET_TIMEOUT 10

#define MAX_SPEED	1.0			// Limit Drive Speed to X%.
#define FF_MAX		2785

#define MAX_SPEED	1.0			// Limit Drive Speed to X%.
const static int REMOTE_0 = 0;
const static int REMOTE_1 = 1;
const static int PID_PRIMARY = 0;
const static int PID_TURN = 1;
const static int SLOT_0 = 0;
const static int SLOT_1 = 1;
const static int SLOT_2 = 2;
const static int SLOT_3 = 3;
struct Gains {
	double kP, kI, kD, kF;
	double kIzone;
	double kPeakOutput;
};

//                                         kP   kI   kD   kF              Iz    PeakOut
constexpr static Gains kGains_Distanc = { 0.1, 0.0,  0.0, 0.0,            100,  0.50 };
constexpr static Gains kGains_Turning = { 2.0, 0.0,  4.0, 0.0,            200,  1.00 };
constexpr static Gains kGains_Velocit = { 0.1, 0.0, 20.0, 1023.0/FF_MAX,  300,  0.50 }; // measured 3200 max velocity
constexpr static Gains kGains_MotProf = { 1.0, 0.0,  0.0, 1023.0/FF_MAX,  400,  1.00 }; // measured 3200 max velocity
constexpr static Gains kGains_MM =      { 0.4, 0.001, 4.0, 1023.0/FF_MAX,  400,  1.00 }; // measured 3200 max velocity

const static int kSlot_Distanc = SLOT_0;
const static int kSlot_Turning = SLOT_1;
const static int kSlot_Velocit = SLOT_2;
const static int kSlot_MotProf = SLOT_3;

#define FF_MAX		2785
const uint8_t kTO = 10;

//Defined functions
#define max( A, B )				A > B ? A : B
#define min( A, B )				A < B ? A : B
#define LimitVal( L, T, H)		max( min( T, H ), L)

const uint8_t kTimeoutMs = 10;		// 10ms for timeouts.
const uint8_t kPIDLoopIdx = 0;

const double 	f = 1023/2785,		// Feed-Forward = 1023 Max Velocity / 3280nu(Max Native Units / 100ms measured).
				p = 0.4,		// Proportional
				i = 0.001, 		// Integral
				d = 4.0;		// Derivative

Drive::Drive(int frontLeftPort,int backLeftPort, int frontRightPort, int backRightPort, int pigeonPort) {
	//inits useful stuff
	pidgey = new PigeonIMU(pigeonPort);
	wasStraightButtonPressed = false;

    // Create & init all drive motors
	frontLeft = new WPI_TalonSRX(frontLeftPort);
	backLeft = new WPI_TalonSRX(backLeftPort);
	frontRight = new WPI_TalonSRX(frontRightPort);
	backRight = new WPI_TalonSRX(backRightPort);

	backLeft->Set(ControlMode::Follower, frontLeft->GetDeviceID());
	backRight->Set(ControlMode::Follower, frontRight->GetDeviceID());

	frontLeft->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,0); //sets the quad encoder as the primary sensor
	frontRight->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,0);

	frontLeft->ConfigOpenloopRamp(0.1, 0.0);
	frontRight->ConfigOpenloopRamp(0.1, 0.0);

	drivetrain = new DifferentialDrive(*frontLeft, *frontRight);

	// Left PID
	frontLeft->Config_kF( PID_PRIMARY, kGains_MM.kF, kTO );
	frontLeft->Config_kP( PID_PRIMARY, kGains_MM.kP, kTO );
	frontLeft->Config_kI( PID_PRIMARY, kGains_MM.kI, kTO);
	frontLeft->Config_kD( PID_PRIMARY, kGains_MM.kD, kTO);
	frontLeft->Config_IntegralZone( PID_PRIMARY, kGains_MM.kIzone, kTO );
	frontLeft->SetInverted(false);
	backLeft->SetInverted(false);

	// Right PID
	frontRight->Config_kF( PID_PRIMARY, kGains_MM.kF, kTO );
	frontRight->Config_kP( PID_PRIMARY, kGains_MM.kP, kTO );
	frontRight->Config_kI( PID_PRIMARY, kGains_MM.kI, kTO);
	frontRight->Config_kD( PID_PRIMARY, kGains_MM.kD, kTO);
	frontRight->Config_IntegralZone( PID_PRIMARY, kGains_MM.kIzone, kTO );
	frontRight->SetInverted(true);
	backRight->SetInverted(true);

	// Setup Motion Magic values.
	double nuSp = 5.0 * 260.9;	// 2ft/s
	frontLeft->ConfigMotionAcceleration( nuSp * 1.5, 0 );
	frontLeft->ConfigMotionCruiseVelocity( nuSp, 0 );
	frontRight->ConfigMotionAcceleration( nuSp * 1.5, 0 );
	frontRight->ConfigMotionCruiseVelocity( nuSp, 0 );

	// Status 10 provides the trajectory target for motion profile AND motion magic.
	frontLeft->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);
	frontRight->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);

	pos = 0;
	vel = 0;
	heading = 0;
	enXfer = false;
	flgRunMP = false;
	cntProfile = 0;

	sPath = "/home/lvuser/Traj/";
	trajLen = 0;
}

void Drive::ArcadeDrive(double speed, double turn){//Drives the drivetrain based on
	drivetrain->ArcadeDrive(speed, turn, false);
}

void Drive::TankDrive(double left, double right){
	drivetrain->TankDrive(left, right, false);
}

void Drive::BenzeneDrive(double throttle, double twist, bool driveStraightButton){
	float steer = 0.0;

	double heading = this->GetYaw();

	// If joystick thumb button for strait driving is pressed...
	if(driveStraightButton) {
		// On button being pressed, reset the gyro heading one time and use that as basis for driving straight.
		if(!wasStraightButtonPressed) {
			this->ResetYaw();
			wasStraightButtonPressed = true; //resets trigger
			heading = 0.0;
		}
		// Dump normal steering and twist inputs. Use gyro to drive strait instead.
		steer = dLimitVal( -0.5, heading * 0.05, 0.5 );
		twist = 0.0;
	}
	// Else, allow turning with joystick and twisting combined.
	else {
		// Else, apply deadband on steering and twist.
		if( fabs(twist) < 0.15 ) twist = 0.0;	// Deadband twist above 25%.
		steer += (twist * 0.50);		// Scale twist down by 50% then add to steer.
		wasStraightButtonPressed = false;				// Reset button press detect flag.
	}

	double leftSpeed = throttle + steer;		// Calculate left and right motor speeds.
	double rightSpeed = throttle - steer;

	drivetrain->TankDrive(leftSpeed, -1.0 * rightSpeed, false);
}

double Drive::InputScale(double value,double power){

	if (value>0){
		return pow(value, power);
	}
	else{
		value = -1.0 * value;
		return -1.0*pow(value, power);
	}
}
void Drive::ResetEncoders(){//Resets Encoders to 0
	frontRight->GetSensorCollection().SetQuadraturePosition(0, 0); //should the timeoutMS be 0?
	frontLeft->GetSensorCollection().SetQuadraturePosition(0, 0);
}

double Drive::GetRightEncoderDistance(){//returns inches travelled by right side of drivetrain
	double rawEncVal = -1.0 * this->GetRightEncoderValue();
	double rightDistanceVal = rawEncVal * INCHES_PER_TICK;
	return rightDistanceVal;
}

double Drive::GetLeftEncoderDistance(){//returns inches travelled by left side of drivetrain
	double rawEncVal = this->GetLeftEncoderValue();
	double leftDistanceVal = rawEncVal * INCHES_PER_TICK;
	return leftDistanceVal;
}

double Drive::GetRightVelocity(){//gets inches per second travelled by left side of drivetrain
	double rate = -1.0 * frontRight->GetSensorCollection().GetQuadratureVelocity();
	double VelocityVal = rate * INCHES_PER_TICK;
	return VelocityVal;
}

double Drive::GetLeftVelocity(){//gets inches per second travelled by right side of drivetrain
	double rate = frontLeft->GetSensorCollection().GetQuadratureVelocity();
	double VelocityVal = rate * INCHES_PER_TICK;
	return VelocityVal;
}

double Drive::GetRightEncoderValue(){
	double rightEncVal = frontRight->GetSensorCollection().GetQuadraturePosition();
	return rightEncVal;
}

double Drive::GetLeftEncoderValue(){
	double leftEncVal = frontLeft->GetSensorCollection().GetQuadraturePosition();
	return leftEncVal;
}

double Drive::GetAverageEncoderValue(){
	double avgEncVal = (this->GetRightEncoderValue() + this->GetLeftEncoderValue())/2.0;
	return avgEncVal;
}

double Drive::GetAverageVelocity(){
	double avgEncVel = (this->GetLeftVelocity() + this->GetRightVelocity())/2.0;
	return avgEncVel;
}

double Drive::GetAverageEncoderDistance(){
	double avgEncDist = (this->GetRightEncoderDistance() + this->GetLeftEncoderDistance())/2.0;
	return avgEncDist;
}

void Drive::ResetYaw(){
	pidgey->SetYaw(0.0,RESET_TIMEOUT);
}
double Drive::GetYaw(){
	double ypr_array[3];
	pidgey->GetYawPitchRoll(ypr_array);
	return ypr_array[0];
}
void Drive::ResetFusedHeading(){
	pidgey->SetFusedHeading(0.0,RESET_TIMEOUT);
}
double Drive::GetFusedHeading(){
	return pidgey->GetFusedHeading();
}
void Drive::SetBrakeMode(){
	frontLeft->SetNeutralMode(NeutralMode::Brake);
	backLeft->SetNeutralMode(NeutralMode::Brake);
	frontRight->SetNeutralMode(NeutralMode::Brake);
	backRight->SetNeutralMode(NeutralMode::Brake);
}
void Drive::SetCoastMode(){
	frontLeft->SetNeutralMode(NeutralMode::Coast);
	backLeft->SetNeutralMode(NeutralMode::Coast);
	frontRight->SetNeutralMode(NeutralMode::Coast);
	backRight->SetNeutralMode(NeutralMode::Coast);
}

double Drive::NormalizeAngle(double angle){
	while (angle < 0){
		angle += 360.0;
	}
	while(angle > 359){
		angle -= 360.0;
	}
	return angle;
}

double Drive::AutonRamping1(double distDifference,double minDriveSpeed,double minDist, double rampRate,double maxDriveSpeed){
	if(distDifference>minDist){
		double autonDriveSpeed = -1.0/(pow(2.0,(rampRate*(distDifference - minDist - (log(maxDriveSpeed - minDriveSpeed)/(log(2.0)*rampRate))))))+maxDriveSpeed;
		return autonDriveSpeed;
	}
	else{
		return minDriveSpeed;
	}
}

double Drive::AutonRamping2(double distDifference,double minSpeed,double midSpeed1, double midSpeed2, double maxSpeed, double changeDist1, double changeDist2, double changeDist3){
	if(distDifference<changeDist1){
		return minSpeed;
	}
	else if(changeDist1<distDifference && distDifference<changeDist2){
		return midSpeed1;
	}
	else if(changeDist2<distDifference && distDifference<changeDist3){
		return midSpeed2;
	}
	else{
		return maxSpeed;
	}
}
void Drive::MotionMagicStraight(double dist){
	double distance = dist*TICKS_PER_INCH;
	frontLeft->Set(ControlMode::MotionMagic,distance);
	frontRight->Set(ControlMode::MotionMagic,distance);
}

void Drive::NeutralizeDrive(){
	frontRight->NeutralOutput();
	frontLeft->NeutralOutput();
}

double Drive::dLimitVal(float low, float test,float high){
	if( test > high ) return high;
	if( test < low ) return low;
	return test;
}
void Drive::MotionMagicTurn(double targetAngle){
	double turnDist = targetAngle * TICKS_PER_DEGREE;
	frontLeft->Set(ControlMode::MotionMagic,turnDist);
	frontRight->Set(ControlMode::MotionMagic,-1.0*turnDist);
}
bool Drive::RunProfile( void ) {
	static uint16_t cnt=0;

	// This is automatically set true in LoadProfile().  When the MP is done,
	// this flag goes false so we don't double execute a MP.  First, use LoadProfile()
	// to run another profile.
	if( flgRunMP == false ) {
		enXfer = false;
		return false;			// We're done.
	}

	// On first time RunProfile is called, enXfer flag will be false.  But
	// flgRunMP will be true from LoadProfile().
	if( (flgRunMP == true) && (enXfer == false) ) {
		cnt = 0;		// Reset counter.
		enXfer = true;	// Start transferring points to the Talons.
		return true;	// Not done yet.
	}

	// Wait approx 100ms for the profile points to start buffering into the Talons.
	// Then, turn on the motors in profile mode!
	if( cnt > 5 ) {
		// Turn on the motion profile in the Talons!
		frontLeft->Set( ControlMode::MotionProfile, 1 );
		frontRight->Set( ControlMode::MotionProfile, 1 );

		MotionProfileStatus mpStatus;
		frontLeft->GetMotionProfileStatus( mpStatus );		// Get the MP status from the Talon.

		// On last MP point in the Talon, the move must be done.
		if( mpStatus.isLast ) {
			enXfer = false;		// Stop the real-time thread activity.
			flgRunMP = false;	// End motion profile running.

			// Switch Talons to holding position mode.
			frontLeft->Set( ControlMode::MotionProfile, 2 );
			frontRight->Set( ControlMode::MotionProfile, 2 );

			// This should not happen - but if it does let people know!
			if( mpStatus.hasUnderrun )
				printf( "Error: Motion Profile UnderRun!\n\n" );

			return false;	// Motion Profile is done.
		}
	}

	cnt += 1;		// Static var used for timing events.
	return true;	// Not done yet.
}


// Given an index to a waypoint group, load the trajectory from the file-system.
// If flgMirror is true, switch sides of the drivetrain to mirror the profile.
// ============================================================================
void Drive::LoadProfile( int idx, bool flgMirror) {
	char s[120];

	enXfer = false;		// My master enable flag for MP buffer xfer.
	flgRunMP = true;	// My master run flag.

	strcpy( s, sPath );
	strcat( s, wp[idx].sTrajLeft );
	printf( "Reading: %s\n", s );
	FILE *fpLf = fopen( s, "r" );
	wp[idx].trajLen = pathfinder_deserialize( fpLf, leftTraj );
	fclose( fpLf );
	trajLen = wp[idx].trajLen;

	strcpy( s, sPath );
	strcat( s, wp[idx].sTrajRight  );
	FILE *fpRt = fopen( s, "r" );
	pathfinder_deserialize( fpRt, rightTraj );
	fclose( fpRt );

	frontLeft->ClearMotionProfileTrajectories();
	frontRight->ClearMotionProfileTrajectories();

	frontLeft->ClearMotionProfileHasUnderrun( 0 );
	frontRight->ClearMotionProfileHasUnderrun( 0 );

	// Fill the top buffer with Talon points.  Note, there is room for
	// about 2048 points for each Talon.  So, don't go too crazy!
	for (int i = 0; i < trajLen; ++i) {
		Segment s = leftTraj[i];
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

		// If flag is true, switch which sides of the drivetrain.
		if( flgMirror )
			frontRight->PushMotionProfileTrajectory( point );
		else
			frontLeft->PushMotionProfileTrajectory( point );

		s = rightTraj[i];
		positionRot = s.position;
		velocityRPM = s.velocity;
		point.position = positionRot * 2607.6;  // Convert ft to nu.
		point.velocity = velocityRPM * 260.76; 	// Convert ft/s to nu/100ms

		// If flag is true, switch which sides of the drivetrain.
		if( flgMirror )
			frontLeft->PushMotionProfileTrajectory( point );
		else
			frontRight->PushMotionProfileTrajectory( point );
	}

	printf( "Loaded %d trajectory points.\n", trajLen );
}

// This task runs at a 5ms rate and keeps the Talon motion profile buffers full
// over the CAN bus.
// ============================================================================
void Drive::mpThread( void ) {

	MotionProfileStatus mpStatus;	// MP Status from a Talon.

	while( 1 ) {

		// If auto mode is true and we're enabled, shovel top buffer points out
		// to the Talons.
		if( frc::RobotState::IsAutonomous() && frc::RobotState::IsEnabled() ) {

			if( enXfer ) {
				// Move points from top-buffer to bottom-buffer.
				frontLeft->ProcessMotionProfileBuffer();
				frontRight->ProcessMotionProfileBuffer();

				frontLeft->GetMotionProfileStatus( mpStatus );		// Get the MP status from the Talon.

				// On last MP point in the Talon, the move is done.
				if( mpStatus.isLast == false ) {
					// Keep counting while not on the last point.
					cntProfile += 1;
				}
			}
		}

		// Sleep the thread for X milliseconds.  Note, there is about 90uS of overhead.
		std::this_thread::sleep_for( std::chrono::microseconds( 5000 - 90 ) );
	}
}
TrajectoryDuration Drive::GetTrajectoryDuration(int durationMs) {
	/* lookup and return valid value */
	switch (durationMs) {
		case 0:		return TrajectoryDuration_0ms;
		case 5:		return TrajectoryDuration_5ms;
		case 10: 	return TrajectoryDuration_10ms;
		case 20: 	return TrajectoryDuration_20ms;
		case 30: 	return TrajectoryDuration_30ms;
		case 40: 	return TrajectoryDuration_40ms;
		case 50: 	return TrajectoryDuration_50ms;
		case 100: 	return TrajectoryDuration_100ms;
	}
	printf("Trajectory Duration not supported - use configMotionProfileTrajectoryPeriod instead\n");
	return TrajectoryDuration_100ms;
}
void Drive::Load_Waypoints(double xBefore,double yBefore,double headingBefore, double xAfter, double yAfter, double headingAfter, int id) {
	// Units are feet and degrees where 90deg points strait ahead.
	// Y+ is forward from where your robot starts.
	// Y- is backward from where your robot starts.
	// X+ is to the right of your robot where it starts.
	// X- is to the left of your robot where it starts.

	wp[id].wpLen = 2;
	wp[id].wps[0] = {xBefore,yBefore,d2r(headingBefore)};
	wp[id].wps[1] = {xAfter,yAfter,d2r(headingAfter)};

	wp[id].vel = 5.0;			// max ft/sec
	wp[id].accel = 15.0;		// max ft/sec^2
	wp[id].jerk = 100.0;		// max ft/sec^3
	std::string name = "Path"+ std::to_string (id);
	std::string left = name + "Lf.bin";
	std::string right = name + "Lf.bin";
	std::string csv = name + "Lf.bin";
	const char *cleft = left.c_str();
	const char *cright = left.c_str();
	const char *ccsv = left.c_str();

	strcpy( wp[id].sTrajLeft, cleft);
	strcpy( wp[id].sTrajRight, cright);
	strcpy( wp[id].sTraj_CSV, ccsv);
}

void Drive::FindPath( int idx ) {
	// Given a small set of waypoints, calculate a bunch of trajectory points that
	// make a smooth curved path for each side of the drivetrain.  The left and
	// right trajectories are saved as binary files.
	// ============================================================================

	TrajectoryCandidate candidate;

	// Prepare the Trajectory for Generation.
	//
	// Arguments:
	// Fit Function:        FIT_HERMITE_CUBIC or FIT_HERMITE_QUINTIC
	// Sample Count:        PATHFINDER_SAMPLES_HIGH (100 000)
	//                      PATHFINDER_SAMPLES_LOW  (10 000)
	//                      PATHFINDER_SAMPLES_FAST (1 000)
	// Time Step:           0.001 Seconds
	// Max Velocity:        15 m/s
	// Max Acceleration:    10 m/s/s
	// Max Jerk:            60 m/s/s/s
//	pathfinder_prepare(  points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	pathfinder_prepare(  wp[idx].wps, wp[idx].wpLen, FIT_HERMITE_CUBIC,
			PATHFINDER_SAMPLES_LOW,
			0.010,			// Sample Rate
			wp[idx].vel,	// Max Velocity ft/s
			wp[idx].accel,	// Max Accel	ft/s^2
			wp[idx].jerk,	// Max Jerk		ft/s^3
			&candidate );

	trajLen = candidate.length;
	wp[idx].trajLen = trajLen;

	// Array of Segments (the trajectory points) to store the trajectory in.
	Segment *trajectory = (Segment*)malloc( trajLen * sizeof(Segment) );

	// Generate the trajectory
	int result = pathfinder_generate(&candidate, trajectory);
	if (result < 0) {
	    // An error occured
	    printf( "Uh-Oh! Trajectory could not be generated!\n" );
	}
	else {
		printf( "Trajectory Length: %d\n", trajLen );
	}

	// Allocate memory for each side of the drivetrain.
	leftTrajectory = (Segment*)malloc( trajLen * sizeof(Segment) );
	rightTrajectory = (Segment*)malloc( trajLen * sizeof(Segment) );

	// The distance between the left and right sides of the wheelbase is 2.0ft.
	double wheelbase_width = 2.0;

	// Generate the Left and Right trajectories of the wheelbase using the
	// originally generated trajectory.
	pathfinder_modify_tank( trajectory, trajLen, leftTrajectory, rightTrajectory, wheelbase_width );

	/*
	double tm=0;
	for (int i = 0; i < 10; i++) {
	    Segment s = leftTrajectory[i];
	    printf( "Time: %f\n", tm += s.dt );
	    printf( "Time Step: %f\n", s.dt );
	    printf( "Coords: (%f, %f)\n", s.x, s.y );
	    printf( "Position (Distance): %f\n", s.position );
	    printf( "Velocity: %f\n", s.velocity );
	    printf( "Acceleration: %f\n", s.acceleration );
	    printf( "Jerk (Acceleration per Second): %f\n", s.jerk );
	    printf( "Heading (radians): %f\n\n", s.heading );
	}
	*/

	char s[120];	// Filename string storage area.

	// Write trajectory to a file on RoboRIO in CSV format.
	// To retrieve use: "scp admin@10.43.84.2:/home/lvuser/Traj/Filename.csv ."
	strcpy( s, sPath );
	strcat( s, wp[idx].sTraj_CSV );
	FILE *fpCSV = fopen( s, "w" );
	pathfinder_serialize_csv( fpCSV, trajectory, trajLen );
	fclose(fpCSV);

	// Save the left side given a filename as a binary trajectory.
	strcpy( s, sPath );
	strcat( s, wp[idx].sTrajLeft );
	FILE *fpLf = fopen( s, "w" );
	pathfinder_serialize( fpLf, leftTrajectory, trajLen );
	fclose(fpLf);

	// Same right side too.
	strcpy( s, sPath );
	strcat( s, wp[idx].sTrajRight );
	FILE *fpRt = fopen( s, "w" );
	pathfinder_serialize( fpRt, rightTrajectory, trajLen );
	fclose(fpRt);

	// Free the memory used by "trajectory" which was the center path of trajectory
	// points.  The trajectory points in leftTrajectory and rightTrajectory are now ready
	// and should be used to drive the robot during auto.
	free( trajectory );
	free( rightTrajectory );
	free( leftTrajectory );
}


//*/
