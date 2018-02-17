/*
 * AutoCommand.h
 *
 *  Created on: Feb 13, 2018
 *      Author: Murali
 */

#ifndef SRC_AUTON_AUTOCOMMAND_H_
#define SRC_AUTON_AUTOCOMMAND_H_

#include <Subsystems/Drive.h>
#include <Subsystems/Intake.h>
#include <Subsystems/Elevator.h>

class AutoCommand {
public:
	AutonDrive *aDrive;
	AutonElevator *aElevator;
	AutonIntake *aIntake;

	AutoCommand();
};

#endif /* SRC_AUTON_AUTOCOMMAND_H_ */
