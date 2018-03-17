/*
 *	BB1 - Benzene Bots FRC 2018 Robot Source Code
 *
 *
 * Motion_Profile.h
 *
 *  Created on: Mar 13, 2018
 *      Author: James Kemp
 */

#ifndef MOTION_PROFILE_H_
#define MOTION_PROFILE_H_

#include "WPILib.h"
#include "ctre/Phoenix.h"

MotionProfileStatus mpStatus;
double _pos = 0, _vel = 0, _heading = 0;	// For active traj. Pt.

bool enXfer = false;

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
