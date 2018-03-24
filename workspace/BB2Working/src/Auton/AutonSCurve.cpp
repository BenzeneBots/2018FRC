/*
 * AutonSCurve.cpp
 *
 *  Created on: Mar 21, 2018
 *      Author: Sanket Nayak
 */

#include <Auton/AutonSCurve.h>


AutonSCurve::AutonSCurve(Drive* robotDrive, int mode,bool side) {
	drive = robotDrive;
	idx= mode;
	escapeTimer = new Timer();
	isRightSide = side;
}
AutonSCurve::~AutonSCurve() {
	// TODO Auto-generated destructor stub
}
void AutonSCurve::Initialize(){
	drive->ResetEncoders();
	drive->ResetFusedHeading();
	drive->ResetAccumulator();
	drive->LoadProfile(idx,isRightSide);
	escapeTimer->Stop();
	escapeTimer->Reset();
}
bool AutonSCurve::Run(){
	if(drive->RunProfile()){
		return true;
		printf("Done running curve! \n");
	}else{
		return false;
	}
}

