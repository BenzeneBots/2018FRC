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


MotionProfileStatus mpStatus;
double _pos = 0, _vel = 0, _heading = 0;	// For active traj. Pt.

bool enXfer = false;
uint32_t cntProfile = 0;

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



// Given an index to a waypoint group, load the trajectory from the file-system.
// ============================================================================
void LoadProfile( int idx ) {
	char s[120];

	strcpy( s, sPath );
	strcat( s, wp[idx].sTrajLeft );
	FILE *fpLf = fopen( s, "r" );
	pathfinder_deserialize( fpLf, leftTraj );
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

		mtrLMaster->PushMotionProfileTrajectory( point );

		s = rightTraj[i];
		positionRot = s.position;
		velocityRPM = s.velocity;
		point.position = positionRot * 2607.6;  // Convert ft to nu.
		point.velocity = velocityRPM * 260.76; 	// Convert ft/s to nu/100ms

		mtrRMaster->PushMotionProfileTrajectory( point );
	}
}






// This task runs at a 5ms rate and keeps the Talon motion profile buffers full
// over the CAN bus.
// ============================================================================
void mpThread( void ) {
	//double tmNow=0;//, tmOld=0;
	//double tmAuto=0;
	//Timer *tm = new Timer();	// Used for computing delta time.
	//static uint16_t cnt=1;		// Don't really need static here since this never returns.
	//bool flgStart = false;		// Flag goes true on auto mode starting.

	//static bool flg=false;
	//static double sp=0.0, dr=0.0;
	//double acl=0.01;

	while( 1 ) {

		//tmNow = tm->GetFPGATimestamp();		// First thing, get the actual timestamp!

		if( frc::RobotState::IsAutonomous() && frc::RobotState::IsEnabled() ) {

			if( enXfer ) {
				// Move points from top-buffer to bottom-buffer.
				mtrLMaster->ProcessMotionProfileBuffer();
				mtrRMaster->ProcessMotionProfileBuffer();

				cntProfile += 1;
			}

			/*
			// Every 200th loop (or once a second), print out some stats.
			if( ++cnt >= 200 ) {
				printf( "AutoTm: %0.2f\n", tmNow - tmAuto );
				cnt = 1;
			}
			*/
		}
		/*
		else {
			tmAuto = tmNow;		// Capture time until autonomous starts.
			cnt = 0;
		}
		*/

		// Sleep the thread for X milliseconds.  Note, there is about 90uS of overhead.
		std::this_thread::sleep_for( std::chrono::microseconds( 5000 - 90 ) );
	}
}

#endif /* MOTION_PROFILE_H_ */
