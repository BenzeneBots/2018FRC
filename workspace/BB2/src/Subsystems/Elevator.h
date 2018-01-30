/*
 * Elevator.h
 *
 *  Created on: Jan 25, 2018
 *      Author: Murali
 */

#ifndef SRC_SUBSYSTEMS_ELEVATOR_H_
#define SRC_SUBSYSTEMS_ELEVATOR_H_

#include <WPILib.h>
#include <ctre/Phoenix.h>

namespace Elevator {

class Elevator {
public:
	Elevator(int);

private:
	TalonSRX *elevatorMotor;


};

} /* namespace Elevator */

#endif /* SRC_SUBSYSTEMS_ELEVATOR_H_ */
