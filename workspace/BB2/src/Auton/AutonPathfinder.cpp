/*
 * AutonPathfinder.cpp
 *
 *  Created on: Mar 20, 2018
 *      Author: Murali
 */

#include <Auton/AutonPathfinder.h>

AutonPathfinder::AutonPathfinder(Drive *robotDrive, int profileNum, bool reversed) {
	drive = robotDrive;
	profileId = profileNum;
	isReversed = reversed;
}

AutonPathfinder::~AutonPathfinder(){

}


void AutonPathfinder::Initialize(){
	drive->LoadProfile(profileId, isReversed);
}

bool AutonPathfinder::Run(){
	return drive->RunProfile();
}

