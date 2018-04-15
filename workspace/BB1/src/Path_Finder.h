#pragma once

#include <Robot.h>
#include <pathfinder.h>


// Global vars used to hold trajectory paths for left and right side.
Segment *leftTrajectory;	// PathFinder() uses malloc to dynamically adjust.
Segment *rightTrajectory;
int trajLen = 0;

const char *sPath = "/home/lvuser/Traj/";

#define MAX_WPS		10	// Max number of waypoints in a path.

// Struct holds a waypoint group.
struct wp {
	Waypoint wps[ MAX_WPS ];		// Allocate Space for up to X Waypoints.
	int wpLen;						// Save how many waypoints to are use of the ten available.
	int trajLen;					// Number of points in the generated trajectory.
	double vel, accel, jerk;		// Max velocity, acceleration, and jerk for this profile.
	char sTrajLeft[32];				// Allocate space to for the Left trajectories filename.
	char sTrajRight[32];			//   "						 Right
	char sTraj_CSV[32];				// Filename for CSV data.
} wp[NUM_PATHS];					// Allocate space for 6 different waypoint groups.



// Fill the waypoint structures with data.
// ============================================================================
void Load_Waypoints() {

	// Units are feet and degrees where 90deg points strait ahead.
	// Y+ is forward from where your robot starts.
	// Y- is backward from where your robot starts.
	// X+ is to the right of your robot where it starts.
	// X- is to the left of your robot where it starts.

	// Side_Switch = Index 0
	// Define Side to Switch.
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	wp[ Side_Switch ].wpLen = 3;
	wp[ Side_Switch ].wps[0] = { 0.0, 	0.0,	d2r(  90 ) };
	wp[ Side_Switch ].wps[1] = { 2.0, 	8.0,	d2r(  90 ) };
	wp[ Side_Switch ].wps[2] = { -3.0, 	10.0,	d2r(  191) };
	wp[ Side_Switch ].vel = 5.0;		// max ft/sec
	wp[ Side_Switch ].accel = 15.0;		// max ft/sec^2
	wp[ Side_Switch ].jerk = 75.0;		// max ft/sec^3
	strcpy( wp[ Side_Switch ].sTrajLeft, "Side_SwitchLf.bin" );
	strcpy( wp[ Side_Switch ].sTrajRight, "Side_SwitchRt.bin" );
	strcpy( wp[ Side_Switch ].sTraj_CSV, "Side_Switch.csv" );


	// Side_Scale = Index 1
	// Define Side to Scale on near side.
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	wp[ Side_Scale ].wpLen = 3;		// Zero points means not yet defined.
	wp[ Side_Scale ].wps[0] = { 0.0, 	0.0,	d2r(  90 ) };
	wp[ Side_Scale ].wps[1] = { 2.0, 	20.0,	d2r(  90 ) };
	wp[ Side_Scale ].wps[2] = { 0.5, 	23.0,	d2r(  180 ) };
	wp[ Side_Scale ].vel = 5.0;			// max ft/sec
	wp[ Side_Scale ].accel = 10.0;		// max ft/sec^2
	wp[ Side_Scale ].jerk = 75.0;		// max ft/sec^3
	strcpy( wp[ Side_Scale ].sTrajLeft, "Side_ScaleLf.bin" );
	strcpy( wp[ Side_Scale ].sTrajRight, "Side_ScaleRt.bin" );
	strcpy( wp[ Side_Scale ].sTraj_CSV, "Side_Scale.csv" );


	// Side_SwitchFar = Index 2
	// Define Side to Switch on Far Opposite Side.
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	wp[ Side_SwitchFar ].wpLen = 5;	//   X       Y         Angle
	wp[ Side_SwitchFar ].wps[0] = { 	0.0,  	0.0, 	d2r(  90 ) };
	wp[ Side_SwitchFar ].wps[1] = { 	8.0, 	12.0, 	d2r(  90 ) };
	wp[ Side_SwitchFar ].wps[2] = { 	8.0,	12.5,	d2r(  90 ) };
	wp[ Side_SwitchFar ].wps[3] = { 	3.5,	17.0,	d2r( 170 ) };
	wp[ Side_SwitchFar ].wps[4] = { 	-6.0,	19.0,	d2r( 180 ) };
	wp[ Side_SwitchFar ].vel = 5.0;			// max ft/sec
	wp[ Side_SwitchFar ].accel = 15.0;		// max ft/sec^2
	wp[ Side_SwitchFar ].jerk = 100.0;		// max ft/sec^3
	strcpy( wp[ Side_SwitchFar ].sTrajLeft, "Side_FarScaleLf.bin" );
	strcpy( wp[ Side_SwitchFar ].sTrajRight, "Side_FarScaleRt.bin" );
	strcpy( wp[ Side_SwitchFar ].sTraj_CSV, "Side_FarScale.csv" );

	// Side_ScaleFar = Index 3
	// Define Side to Scale on far side.
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	wp[ Side_ScaleFar ].wpLen = 6;		// Zero points means not yet defined.
	wp[ Side_ScaleFar ].wps[0] = { 0.0, 	0.0,	d2r(  90 ) };
	wp[ Side_ScaleFar ].wps[1] = { 1.0, 	12.5,	d2r(  90 ) };
	wp[ Side_ScaleFar ].wps[2] = { -3.0, 	15.5,	d2r(  180 ) };
	wp[ Side_ScaleFar ].wps[3] = { -12.0, 	16.0,	d2r(  190 ) };
	wp[ Side_ScaleFar ].wps[4] = { -16.0, 	18.0,	d2r(  95 ) };
	wp[ Side_ScaleFar ].wps[5] = { -16.0, 	19.0,	d2r(  83 ) };
	wp[ Side_ScaleFar ].vel = 5.0;			// max ft/sec
	wp[ Side_ScaleFar ].accel = 10.0;		// max ft/sec^2
	wp[ Side_ScaleFar ].jerk = 75.0;		// max ft/sec^3
	strcpy( wp[ Side_ScaleFar ].sTrajLeft, "Side_ScaleFarLf.bin" );
	strcpy( wp[ Side_ScaleFar ].sTrajRight, "Side_ScaleFarRt.bin" );
	strcpy( wp[ Side_ScaleFar ].sTraj_CSV, "Side_ScaleFar.csv" );

	// Mid_SwitchLeft = Index 4
	// Define Mid starting to switch on left side.
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	wp[ Mid_SwitchLeft ].wpLen = 4;		// Zero points means not yet defined.
	wp[ Mid_SwitchLeft ].wps[0] = { 0.0, 	0.0,	d2r(  90 ) };
	wp[ Mid_SwitchLeft ].wps[1] = { 0.0, 	0.8,	d2r(  90 ) };
	wp[ Mid_SwitchLeft ].wps[2] = { -3.0, 	2.0,	d2r( 180 ) };	// -3, 2
	wp[ Mid_SwitchLeft ].wps[3] = { -7.5,	6.0,	d2r( 95 ) };	// -6, 6
	wp[ Mid_SwitchLeft ].vel = 5.0;			// max ft/sec
	wp[ Mid_SwitchLeft ].accel = 5.0;		// max ft/sec^2
	wp[ Mid_SwitchLeft ].jerk = 75.0;		// max ft/sec^3
	strcpy( wp[ Mid_SwitchLeft ].sTrajLeft, "Mid_LeftSwitchLf.bin" );
	strcpy( wp[ Mid_SwitchLeft ].sTrajRight, "Mid_LeftSwitchRt.bin" );
	strcpy( wp[ Mid_SwitchLeft ].sTraj_CSV, "Mid_LeftSwitch.csv" );

	// Mid_SwitchRight = Index 5
	// Define Mid to Right Switch.
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	wp[ Mid_SwitchRight ].wpLen = 2;
	wp[ Mid_SwitchRight ].wps[0] = { 0, 	0, 		d2r(  90 ) };
	wp[ Mid_SwitchRight ].wps[1] = { 4.2,	7.9,	d2r(  90 ) };
	wp[ Mid_SwitchRight ].vel = 5.0;			// max ft/sec
	wp[ Mid_SwitchRight ].accel = 10.0;		// max ft/sec^2
	wp[ Mid_SwitchRight ].jerk = 100.0;		// max ft/sec^3
	strcpy( wp[ Mid_SwitchRight ].sTrajLeft, "Mid_RightSwitchLf.bin" );
	strcpy( wp[ Mid_SwitchRight ].sTrajRight, "Mid_RightSwitchRt.bin" );
	strcpy( wp[ Mid_SwitchRight ].sTraj_CSV, "Mid_RightSwitch.csv" );

	// Left_SwitchMid = Index 6
	// Define Left to Mid Cube Pile.
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	wp[ Left_SwitchMid ].wpLen = 2;
	wp[ Left_SwitchMid ].wps[0] = { 0, 	0, 		d2r(  90 ) };
	wp[ Left_SwitchMid ].wps[1] = { 5.5, -5.0,	d2r(  88 ) };
	wp[ Left_SwitchMid ].vel = 5.0;			// max ft/sec
	wp[ Left_SwitchMid ].accel = 10.0;		// max ft/sec^2
	wp[ Left_SwitchMid ].jerk = 75.0;		// max ft/sec^3
	strcpy( wp[ Left_SwitchMid ].sTrajLeft, "Left_MidSwitchLf.bin" );
	strcpy( wp[ Left_SwitchMid ].sTrajRight, "Left_MidSwitchRt.bin" );
	strcpy( wp[ Left_SwitchMid ].sTraj_CSV, "Left_MidSwitch.csv" );


	// Right_SwitchMid = Index 7
	// Define Right to Mid Switch.
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	wp[ Right_SwitchMid ].wpLen = 2;
	wp[ Right_SwitchMid ].wps[0] = { 0, 	0, 		d2r(  90 ) };
	wp[ Right_SwitchMid ].wps[1] = { -4.3,	-5.0,	d2r(  95 ) };
	wp[ Right_SwitchMid ].vel = 5.0;			// max ft/sec
	wp[ Right_SwitchMid ].accel = 10.0;		// max ft/sec^2
	wp[ Right_SwitchMid ].jerk = 100.0;		// max ft/sec^3
	strcpy( wp[ Right_SwitchMid ].sTrajLeft, "Right_MidSwitchLf.bin" );
	strcpy( wp[ Right_SwitchMid ].sTrajRight, "Right_MidSwitchRt.bin" );
	strcpy( wp[ Right_SwitchMid ].sTraj_CSV, "Right_MidSwitch.csv" );

	// Switch_Cube = Index 8
	// Define switch to cube on back wall.
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	wp[ Switch_Cube ].wpLen = 4;
	wp[ Switch_Cube ].wps[0] = { 0, 	0, 		d2r(  90 ) };
	wp[ Switch_Cube ].wps[1] = { -1.0,	-1.37,	d2r( 45 ) };
	wp[ Switch_Cube ].wps[2] = { -3.0,	-1.87,	d2r(  160 ) };
	wp[ Switch_Cube ].wps[3] = { -5.87,	4.13,	d2r(  -8 ) };
	wp[ Switch_Cube ].vel = 5.0;			// max ft/sec
	wp[ Switch_Cube ].accel = 10.0;		// max ft/sec^2
	wp[ Switch_Cube ].jerk = 100.0;		// max ft/sec^3
	strcpy( wp[ Switch_Cube ].sTrajLeft, "Switch_CubeLf.bin" );
	strcpy( wp[ Switch_Cube ].sTrajRight, "Switch_CubeRt.bin" );
	strcpy( wp[ Switch_Cube ].sTraj_CSV, "Switch_Cube.csv" );


}



// Given a small set of waypoints, calculate a bunch of trajectory points that
// make a smooth curved path for each side of the drivetrain.  The left and
// right trajectories are saved as binary files on the RoboRIO file-system.
// ============================================================================
void PathFinder( int idx ) {

	// Verify the index is valid befor trying to use it!
	if( (idx < 0) || (idx >= NUM_PATHS) ) {
		printf( "Error: PathFinder index out of bounds.\n" );
		return;
	}

	// Verify at least two waypoints are given to pathFinder.  Can't
	// calculate a path if zero or one waypoint are given.
	if( wp[idx].wpLen < 2 ) {
		printf( "Warning: Pathfinder needs at least two waypoints.\n" );
		printf( "    Number of Waypoints given: %d\n", wp[idx].wpLen );
		return;
	}

	// Verify we're not being asked to index beyond whats allocated.
	if( wp[idx].wpLen >= MAX_WPS ) {
		printf( "Error: PathFinder max number of waypoints exceded\n" );
		printf( "    Given %d waypoints when only 2 to %d allowed.\n", wp[idx].wpLen, MAX_WPS );
		return;
	}

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
	    printf( "Error: Uh-Oh! Trajectory could not be generated!\n" );
	}
	else {
		printf( "     Trajectory Length: %d\n", trajLen );
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
	// To retrieve all use: "scp admin@10.43.84.2:/home/lvuser/Traj/*.csv Traj/."
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
