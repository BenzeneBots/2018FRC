
#ifndef PATH_FINDER_H_
#define PATH_FINDER_H_

#pragma once

#include <pathfinder.h>

// Global vars used to hold trajectory paths for left and right side.
Segment *leftTrajectory;	// PathFinder() uses malloc to dynamically adjust.
Segment *rightTrajectory;
int trajLen = 0;

// Given a small set of waypoints, calculate a bunch of trajectory points that
// make a smooth curved path for each side of the drivetrain.
// ============================================================================
void PathFinder(double distX1, double distY1,double distX2, double distY2,double angBefore,double angAfter) {
	int POINT_LENGTH = 2;

	Waypoint points[POINT_LENGTH];

	// Units are feet and degrees where 90deg points strait ahead.
	// X+ is forward from where your robot starts.
	// X- is backward from where your robot starts.
	// Y+ is to the right of your robot where it starts.
	// Y- is to the left of your robot where it starts.

	//               Xft   Yft  Theta
	Waypoint p1 = {distX1/12,distY1/12,d2r(angBefore)};
	Waypoint p2 = {distX2/12,distY2/12,d2r(angAfter) };
	points[0] = p1;
	points[1] = p2;


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
	pathfinder_prepare(  points, POINT_LENGTH, FIT_HERMITE_CUBIC,

			PATHFINDER_SAMPLES_LOW,
			0.010,			// Sample Rate
			3.0,			// Max Velocity ft/s
			3.0, 			// Max Accel	ft/s/s
			30.0, 			// Max Jerk		ft/s/s/s
			&candidate );

	trajLen = candidate.length;

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

	// The distance between the left and right sides of the wheelbase is 2.5ft.
	double wheelbase_width = 23.75/12;

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

	// Write trajectory to temp file on RoboRIO in CSV format.
	// To retrieve use: "scp admin@10.43.84.2:/tmp/myTraj.csv ."
	FILE *fp = fopen("/tmp/myTraj.csv", "w");
	pathfinder_serialize_csv(fp, trajectory, trajLen);
	fclose(fp);

	// Free the memory used by "trajectory" which was the center path of trajectory
	// points.  The trajectory points in leftTrajectory and rightTrajectory are now ready
	// and should be used to drive the robot during auto.
	free( trajectory );
}
#endif /* PATH_FINDER_H_ */
