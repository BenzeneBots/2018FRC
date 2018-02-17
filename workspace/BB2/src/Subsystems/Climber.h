/*
 * Climber.h
 *
 *  Created on: Feb 8, 2018
 *      Author: Murali
 */

#ifndef SRC_SUBSYSTEMS_CLIMBER_H_
#define SRC_SUBSYSTEMS_CLIMBER_H_

#include <WPILib.h>

namespace Climber {

class Climber {
public:
	Climber(int);
	void SpoolClimber(bool);

private:
	Victor *climberMotor;
};

class AutonClimber : public Climber {

};

} /* namespace Climber */

#endif /* SRC_SUBSYSTEMS_CLIMBER_H_ */
