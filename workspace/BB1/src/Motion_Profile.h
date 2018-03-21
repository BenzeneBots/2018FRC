/*
 * Motion_Profile.h
 *
 *  Created on: Mar 13, 2018
 *      Author: James Kemp
 */

#ifndef MOTION_PROFILE_H_
#define MOTION_PROFILE_H_

#include "WPILib.h"
#include "ctre/Phoenix.h"
#include <pathfinder.h>

TrajectoryDuration GetTrajectoryDuration( int durationMs );

MotionProfileStatus mpStatus;
double _pos = 0, _vel = 0, _heading = 0;	// For active traj. Pt.

bool enXfer = false;
uint32_t cntProfile = 0;
bool flgRunMP = false;

// Call this function often when running a motion profile.  This function will
// return true when the profile has ended.
// ============================================================================
bool RunProfile( void ) {
	static uint16_t cnt=0;

	// This is automatically set true in LoadProfile().  When the MP is done,
	// this flag goes false so we don't double execute a MP.  First, use LoadProfile()
	// to run another profile.
	if( flgRunMP == false ) {
		enXfer = false;
		return true;			// We're done.
	}

	// On first time RunProfile is called, enXfer flag will be false.  But
	// flgRunMP will be true from LoadProfile().
	if( (flgRunMP == true) && (enXfer == false) ) {
		cnt = 0;		// Reset counter.
		enXfer = true;	// Start transferring points to the Talons.
		return false;	// Not done yet.
	}

	// Wait approx 100ms for the profile points to start buffering into the Talons.
	// Then, turn on the motors in profile mode!
	if( cnt > 5 ) {
		// Turn on the motion profile in the Talons!
		mtrLMaster->Set( ControlMode::MotionProfile, 1 );
		mtrRMaster->Set( ControlMode::MotionProfile, 1 );

		MotionProfileStatus mpStatus;
		mtrLMaster->GetMotionProfileStatus( mpStatus );		// Get the MP status from the Talon.

		// On last MP point in the Talon, the move must be done.
		if( mpStatus.isLast ) {
			enXfer = false;		// Stop the real-time thread activity.
			flgRunMP = false;	// End motion profile running.

			// Switch Talons to holding position mode.
			mtrLMaster->Set( ControlMode::MotionProfile, 2 );
			mtrRMaster->Set( ControlMode::MotionProfile, 2 );

			// This should not happen - but if it does let people know!
			if( mpStatus.hasUnderrun )
				printf( "Error: Motion Profile UnderRun!\n\n" );

			return true;	// Motion Profile is done.
		}
	}

	cnt += 1;		// Static var used for timing events.
	return false;	// Not done yet.
}


// Given an index to a waypoint group, load the trajectory from the file-system.
// If flgMirror is true, switch sides of the drivetrain to mirror the profile.
// ============================================================================
void LoadProfile( int idx, bool flgMirror ) {
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

	mtrLMaster->ClearMotionProfileTrajectories();
	mtrRMaster->ClearMotionProfileTrajectories();

	mtrLMaster->ClearMotionProfileHasUnderrun( 0 );
	mtrRMaster->ClearMotionProfileHasUnderrun( 0 );

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
			mtrRMaster->PushMotionProfileTrajectory( point );
		else
			mtrLMaster->PushMotionProfileTrajectory( point );

		s = rightTraj[i];
		positionRot = s.position;
		velocityRPM = s.velocity;
		point.position = positionRot * 2607.6;  // Convert ft to nu.
		point.velocity = velocityRPM * 260.76; 	// Convert ft/s to nu/100ms

		// If flag is true, switch which sides of the drivetrain.
		if( flgMirror )
			mtrLMaster->PushMotionProfileTrajectory( point );
		else
			mtrRMaster->PushMotionProfileTrajectory( point );
	}

	printf( "Loaded %d trajectory points.\n", trajLen );
}

// This task runs at a 5ms rate and keeps the Talon motion profile buffers full
// over the CAN bus.
// ============================================================================
void mpThread( void ) {

	MotionProfileStatus mpStatus;	// MP Status from a Talon.

	while( 1 ) {

		// If auto mode is true and we're enabled, shovel top buffer points out
		// to the Talons.
		if( frc::RobotState::IsAutonomous() && frc::RobotState::IsEnabled() ) {

			if( enXfer ) {
				// Move points from top-buffer to bottom-buffer.
				mtrLMaster->ProcessMotionProfileBuffer();
				mtrRMaster->ProcessMotionProfileBuffer();

				mtrLMaster->GetMotionProfileStatus( mpStatus );		// Get the MP status from the Talon.

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




/**
 * Find enum value if supported.
 * @param durationMs
 * @return enum equivalent of durationMs
 */
TrajectoryDuration GetTrajectoryDuration(int durationMs) {
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



#endif /* MOTION_PROFILE_H_ */
