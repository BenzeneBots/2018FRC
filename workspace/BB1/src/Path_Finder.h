#pragma once

#include <pathfinder.h>

// Global vars used to hold trajectory paths for left and right side.
Segment *leftTrajectory;	// PathFinder() uses malloc to dynamically adjust.
Segment *rightTrajectory;
int trajLen = 0;

// Units are feet and degrees where 90deg points strait ahead.
// Y+ is forward from where your robot starts.
// Y- is backward from where your robot starts.
// X+ is to the right of your robot where it starts.
// X- is to the left of your robot where it starts.

const char *sPath = "/home/lvuser/Traj/";

// Each waypoint group has a name defined in this enum.
enum paths {
	Side_Switch, Side_Scale, Side_SwitchFar,
	Side_ScaleFar, Mid_SwitchLeft, 	Mid_SwitchRight };
typedef paths paths;

// Struct holds a waypoint group.
struct wp {
	Waypoint wps[10];				// Allocate Space for up to 10 Waypoints.
	int wpLen;						// Save how many waypoints to are use of the ten available.
	int trajLen;					// Number of points in the generated trajectory.
	double vel, accel, jerk;		// Max velocity, acceleration, and jerk for this profile.
	char sTrajLeft[32];				// Allocate space to for the Left trajectories filename.
	char sTrajRight[32];			//   "						 Right
	char sTraj_CSV[32];				// Filename for CSV data.
} wp[6];							// Allocate space for 6 different waypoint groups.





// Fill the waypoint structures with data.
// ============================================================================
void Load_Waypoints() {
	// Define Side to Switch on Far Opposite Side.
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	wp[Side_SwitchFar].wpLen = 5;
	wp[Side_SwitchFar].wps[0] = { 0.0,  0.0, 	d2r(  90 ) };
	wp[Side_SwitchFar].wps[1] = { 8, 	12, 	d2r(  90 ) };
	wp[Side_SwitchFar].wps[2] = { 8,	12.5,	d2r(  90 ) };
	wp[Side_SwitchFar].wps[3] = { 3.5,	17,		d2r( 170 ) };
	wp[Side_SwitchFar].wps[4] = { -6,	19,		d2r( 180 ) };
	wp[Side_SwitchFar].vel = 5.0;			// max ft/sec
	wp[Side_SwitchFar].accel = 15.0;		// max ft/sec^2
	wp[Side_SwitchFar].jerk = 100.0;		// max ft/sec^3
	strcpy( wp[Side_SwitchFar].sTrajLeft, "Side_FarScaleLf.bin" );
	strcpy( wp[Side_SwitchFar].sTrajRight, "Side_FarScaleRt.bin" );
	strcpy( wp[Side_SwitchFar].sTraj_CSV, "Side_FarScale.csv" );

	// Define Mid to Right Switch.
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	wp[Mid_SwitchRight].wpLen = 3;
	wp[Mid_SwitchRight].wps[0] = { 0, 	0, 		d2r(  90 ) };
	wp[Mid_SwitchRight].wps[1] = { 3, 	3, 		d2r(  60 ) };
	wp[Mid_SwitchRight].wps[2] = { 5.5,	7.5,	d2r(  87 ) };
	wp[Mid_SwitchRight].vel = 5.0;			// max ft/sec
	wp[Mid_SwitchRight].accel = 10.0;		// max ft/sec^2
	wp[Mid_SwitchRight].jerk = 75.0;		// max ft/sec^3
	strcpy( wp[Mid_SwitchRight].sTrajLeft, "Mid_RightSwitchLf.bin" );
	strcpy( wp[Mid_SwitchRight].sTrajRight, "Mid_RightSwitchRt.bin" );
	strcpy( wp[Mid_SwitchRight].sTraj_CSV, "Mid_RightSwitch.csv" );
}



// Given a small set of waypoints, calculate a bunch of trajectory points that
// make a smooth curved path for each side of the drivetrain.  The left and
// right trajectories are saved as binary files.
// ============================================================================
void PathFinder( int idx ) {

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
