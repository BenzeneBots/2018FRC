/*
 * Elevator.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: Murali
 */

#include <Subsystems/Elevator.h>

//constants
#define INCHES_PER_TICK 0.04125//need to actually measure

namespace Elevator {

Elevator::Elevator(int elevatorPort) {
	elevatorMotor = new TalonSRX(elevatorPort);
	// TODO Auto-generated constructor stub
}

//void Elevator::LiftElevatorToHeight(){
//}



} /* namespace Elevator */
