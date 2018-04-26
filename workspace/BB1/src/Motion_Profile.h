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

#define FTTONU					2607.6		// Conversion factor for ft to nu.
#define FTPERSECTONUPER100MS	260.76		// Conversion factor for Ft/sec to NU/100ms.

TrajectoryDuration GetTrajectoryDuration( int durationMs );

MotionProfileStatus mpStatus;
double _pos = 0, _vel = 0, _heading = 0;	// For active traj. Pt.

bool enXfer = false;
uint32_t cntProfile = 0;
uint32_t cntAuto = 0;
bool flgRunMP = false;

// Call this function often when running a motion profile.  This function will
// return false when the profile has ended.
// ============================================================================
bool RunProfile( void ) {
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
		if( frc::RobotState::IsEnabled() ) {
			// Turn on the motion profile in the Talons!
			mtrLMaster->Set( ControlMode::MotionProfile, 1 );
			mtrRMaster->Set( ControlMode::MotionProfile, 1 );
		}

		MotionProfileStatus mpStatus;
		mtrLMaster->GetMotionProfileStatus( mpStatus );		// Get the MP status from the Talon.

		// On last MP point in the Talon, the move must be done.
		if( mpStatus.isLast || (frc::RobotState::IsEnabled() == false) ) {
			enXfer = false;		// Stop the real-time thread activity.
			flgRunMP = false;	// End motion profile running.

			// Switch Talons to holding position mode.
			mtrLMaster->Set( ControlMode::MotionProfile, 2 );
			mtrRMaster->Set( ControlMode::MotionProfile, 2 );

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
void LoadProfile( int idx, bool flgMirror, bool flgReverse) {
	char s[120];
	double dir=1.0;		// Assume forward.

	#ifdef PRACTICE_BOT
		flgMirror = !flgMirror;
	#endif

	// Automatically mirror if running backwards.
	if( flgReverse ) flgMirror = !flgMirror;

	printf( "Load Profile started...\n" );
	enXfer = false;		// My master enable flag for MP buffer xfer.
	flgRunMP = true;	// My master run flag.

	strcpy( s, sPath );
	strcat( s, wp[idx].sTrajLeft );
	printf( "Reading Index: %d -> %s\n", idx, s );
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

	mtrLMaster->SetIntegralAccumulator( 0, 0, 0 );
	mtrRMaster->SetIntegralAccumulator( 0, 0, 0 );

	mtrLMaster->SetSelectedSensorPosition(0,0,0);
	mtrRMaster->SetSelectedSensorPosition(0,0,0);

	if( flgReverse ) dir = -1.0;

	// Fill the top buffer with Talon points.  Note, there is room for
	// about 2048 points for each Talon.  So, don't go too crazy!
	for (int i = 0; i < trajLen; ++i) {
		Segment s = leftTraj[i];
		double positionRot = s.position;
		double velocityRPM = s.velocity;

		TrajectoryPoint point;

		/* for each point, fill our structure and pass it to API */
		point.position = positionRot * FTTONU * dir;  				// Convert ft to nu.
		point.velocity = velocityRPM * FTPERSECTONUPER100MS * dir; 	// Convert ft/s to nu/100ms
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
		point.position = positionRot * FTTONU * dir;  				// Convert ft to nu.
		point.velocity = velocityRPM * FTPERSECTONUPER100MS * dir; 	// Convert ft/s to nu/100ms

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

			cntAuto += 1;	// General counter for counting auto mode total.
		}
		else {
			// In any other mode, set the flag to false. Therefore, next time we
			// enter auto the flag will be defaulted off and WAIT to be set.
			enXfer = false;
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
