/*
 * AutonMoveElevatorToHeight.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: Murali
 */

#include <Auton/AutonMoveElevatorToHeight.h>

AutonMoveElevatorToHeight::AutonMoveElevatorToHeight(Elevator *elevator, double heightTarget) {
	robotElevator = elevator;
	targetHeight = heightTarget;
}

AutonMoveElevatorToHeight::~AutonMoveElevatorToHeight() {
	// TODO Auto-generated destructor stub
}

void AutonMoveElevatorToHeight::Initialize(){
	robotElevator->SetElevatorTarget(targetHeight);
}

bool AutonMoveElevatorToHeight::Run(){
	return robotElevator->MoveElevator(0.0);
}

