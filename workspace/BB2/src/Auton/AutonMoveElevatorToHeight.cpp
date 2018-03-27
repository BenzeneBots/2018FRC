/*
 * AutonMoveElevatorToHeight.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: Murali
 */

#include <Auton/AutonMoveElevatorToHeight.h>

AutonMoveElevatorToHeight::AutonMoveElevatorToHeight(Elevator *robotElevator, double heightTarget) {
	elevator = robotElevator;
	targetHeight = heightTarget;
}

AutonMoveElevatorToHeight::~AutonMoveElevatorToHeight() {
	// TODO Auto-generated destructor stub
}

void AutonMoveElevatorToHeight::Initialize(){
	elevator->SetElevatorTarget(targetHeight);
}

bool AutonMoveElevatorToHeight::Run(){
	elevator->MoveElevator(0.0);
	printf("Height: %f\n", elevator->GetElevatorHeight());
	if(elevator->GetElevatorPosition() >= targetHeight){
		elevator->SetToOutput(0.0);
		printf("Finished\n");
		return true;
	}
	return false;
}

