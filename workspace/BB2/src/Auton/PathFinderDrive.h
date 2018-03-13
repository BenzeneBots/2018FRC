/*
 * PathFinderDrive.h
 *
 *  Created on: Mar 9, 2018
 *      Author: Sanket Nayak
 */

#ifndef SRC_AUTON_PATHFINDERDRIVE_H_
#define SRC_AUTON_PATHFINDERDRIVE_H_

#include <ctre/Phoenix.h>
#include <WPILib.h>
#include <Subsystems/Drive.h>

class PathFinderDrive {
public:
	PathFinderDrive(Drive*);
};

#endif /* SRC_AUTON_PATHFINDERDRIVE_H_ */
