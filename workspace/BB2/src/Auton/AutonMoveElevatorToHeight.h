/*
 * AutonMoveElevatorToHeight.h
 *
 *  Created on: Mar 8, 2018
 *      Author: Murali
 */

#ifndef SRC_AUTON_AUTONMOVEELEVATORTOHEIGHT_H_
#define SRC_AUTON_AUTONMOVEELEVATORTOHEIGHT_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Elevator.h>

class AutonMoveElevatorToHeight: public AutoCommand {
public:
	AutonMoveElevatorToHeight(Elevator*, double);
	virtual ~AutonMoveElevatorToHeight();

	void Initialize();
	bool Run();

private:
	Elevator *elevator;
	double targetHeight;
};

#endif /* SRC_AUTON_AUTONMOVEELEVATORTOHEIGHT_H_ */
