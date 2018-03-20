/*
 * PathFinder.h
 *
 *  Created on: Mar 20, 2018
 *      Author: Sanket Nayak
 */

#ifndef SRC_AUTON_PATHFINDER_H_
#define SRC_AUTON_PATHFINDER_H_


#define NUM_PATHS	6	// There are six possible paths in Auto mode.
#define MAX_WPS		10	// Max number of waypoints in a path.

#pragma once

#include <pathfinder.h>

// Each waypoint group has a name defined in this enum.
enum paths {
	Side_Switch, Side_Scale, Side_SwitchFar,
	Side_ScaleFar, Mid_SwitchLeft, 	Mid_SwitchRight };
typedef paths paths;

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
	wp[ Side_Switch ].wps[1] = { 2.0, 	5.0,	d2r(  60 ) };
	wp[ Side_Switch ].wps[2] = { -1.0, 	14.0,	d2r(  180 ) };
	wp[ Side_Switch ].vel = 5.0;			// max ft/sec
	wp[ Side_Switch ].accel = 10.0;		// max ft/sec^2
	wp[ Side_Switch ].jerk = 75.0;		// max ft/sec^3
	strcpy( wp[ Side_Switch ].sTrajLeft, "Side_SwitchLf.bin" );
	strcpy( wp[ Side_Switch ].sTrajRight, "Side_SwitchRt.bin" );
	strcpy( wp[ Side_Switch ].sTraj_CSV, "Side_Switch.csv" );


	// Side_Scale = Index 1
	// Define Side to Scale on near side.
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	wp[ Side_Scale ].wpLen = 0;		// Zero points means not yet defined.

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
	wp[ Side_ScaleFar ].wpLen = 0;		// Zero points means not yet defined.

	// Mid_SwitchLeft = Index 4
	// Define Mid starting to switch on left side.
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	wp[ Mid_SwitchLeft ].wpLen = 0;		// Zero points means not yet defined.

	// Mid_SwitchRight = Index 5
	// Define Mid to Right Switch.
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	wp[ Mid_SwitchRight ].wpLen = 3;
	wp[ Mid_SwitchRight ].wps[0] = { 0, 	0, 		d2r(  90 ) };
	wp[ Mid_SwitchRight ].wps[1] = { 3, 	3, 		d2r(  60 ) };
	wp[ Mid_SwitchRight ].wps[2] = { 5.5,	7.5,	d2r(  87 ) };
	wp[ Mid_SwitchRight ].vel = 5.0;			// max ft/sec
	wp[ Mid_SwitchRight ].accel = 10.0;		// max ft/sec^2
	wp[ Mid_SwitchRight ].jerk = 75.0;		// max ft/sec^3
	strcpy( wp[ Mid_SwitchRight ].sTrajLeft, "Mid_RightSwitchLf.bin" );
	strcpy( wp[ Mid_SwitchRight ].sTrajRight, "Mid_RightSwitchRt.bin" );
	strcpy( wp[ Mid_SwitchRight ].sTraj_CSV, "Mid_RightSwitch.csv" );
}


#endif /* SRC_AUTON_PATHFINDER_H_ */
