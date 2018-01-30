/*
 * Elevator.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: Murali
 */

#include <Subsystems/Elevator.h>

namespace Elevator {

Elevator::Elevator(int elevatorPort) {
	elevatorMotor = new TalonSRX(elevatorPort);
	// TODO Auto-generated constructor stub
}



} /* namespace Elevator */
