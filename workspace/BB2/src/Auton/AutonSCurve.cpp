/*
 * AutonSCurve.cpp
 *
 *  Created on: Mar 17, 2018
 *      Author: Sanket Nayak
 */

#include <Auton/AutonSCurve.h>
#include <Subsystems/Drive.h>

#include <Auton/Motion_Profile.h>
#include <Auton/Path_Finder.h>

AutonSCurve::AutonSCurve(Drive* driveTrain, double X1Distt, double Y1Distt,double Ang1t,double X2Distt, double Y2Distt, double Ang2t) {
	drive = driveTrain;
	X1Dist = X1Distt;
	Y1Dist = Y1Distt;
	Ang1 = Ang1t;
	X2Dist = X2Distt;
	Y2Dist = Y2Distt;
	Ang2 = Ang2t;
}
AutonSCurve::~AutonSCurve() {
	// TODO Auto-generated destructor stub
}
void AutonSCurve::Initialize(){
	//PathFinder(X1Dist,Y1Dist,X2Dist,Y2Dist,Ang1,Ang2);
	drive->AutonPrep();

	// Fill the top buffer with Talon points.  Note, there is room for
	// about 2048 points for each Talon.  So, don't go too crazy!
	for (int i = 0; i < trajLen; ++i) {
		Segment s = leftTrajectory[i];
		double positionRot = s.position;
		double velocityRPM = s.velocity;

		TrajectoryPoint point;

		// for each point, fill our structure and pass it to API
		point.position = positionRot * 2607.6;  // Convert ft to nu.
		point.velocity = velocityRPM * 260.76; 	// Convert ft/s to nu/100ms
		point.headingDeg = 0; //future feature - not used in this example
		point.profileSlotSelect0 = 0; // which set of gains would you like to use [0,3]?
		point.profileSlotSelect1 = 0; // future feature  - not used in this example - cascaded PID [0,1], leave zero
		point.timeDur = GetTrajectoryDuration((int) s.dt );

		// Set true on first point.
		point.zeroPos = (i == 0) ? true : false;
		// Set true on last point.
		point.isLastPoint = ((i+1) == trajLen) ? true : false;

		drive->LeftTraj(point);

		s = rightTrajectory[i];
		positionRot = s.position;
		velocityRPM = s.velocity;
		point.position = positionRot * 2607.6;  // Convert ft to nu.
		point.velocity = velocityRPM * 260.76; 	// Convert ft/s to nu/100ms

		drive->RightTraj(point);

	}

	enXfer = true;
	printf( "Auto Pts Running...\n" );
}
bool AutonSCurve::Run(){
	printf("Trying to run... /n");
	static uint16_t cnt=0;
	if( enXfer ) {
			if( cnt > 5 ) {
				drive->AutonMotionProfile(1);
			}
		}
		else {
			cnt = 0;
		}

		MotionProfileStatus mpStatus;
		drive->AutonProfileStatus(mpStatus);

		if( enXfer && mpStatus.isLast ) {
			enXfer = false;
			cnt = 0;
			drive->AutonMotionProfile(2);
			printf( "Motion Profile Finished\n" );

			if( mpStatus.hasUnderrun ) {
				printf( "\n****** Motion Profile UnderRun !!!!!!!\n\n" );
			}
		}

		if( mpStatus.isLast && cnt > 20 ) {
			drive->AutonStop();
			return true;
		}else{
			return false;
		}

		cnt += 1;

}
