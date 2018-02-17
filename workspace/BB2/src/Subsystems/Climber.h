/*
 * Climber.h
 *
 *  Created on: Feb 8, 2018
 *      Author: Murali
 */

#ifndef SRC_SUBSYSTEMS_CLIMBER_H_
#define SRC_SUBSYSTEMS_CLIMBER_H_

#include <WPILib.h>

class Climber {
public:
	Climber(int);
	void SpoolClimber(bool);

private:
	Victor *climberMotor;
};

#endif /* SRC_SUBSYSTEMS_CLIMBER_H_ */
