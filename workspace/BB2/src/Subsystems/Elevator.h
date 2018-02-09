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
	Elevator(int,int,int);
	void SetEncoderPosition(int);
	double GetElevatorPosition();
	double GetElevatorRate();
	void SetToOutput(double);
	void EnableSoftLimits();
	void SetElevatorSetPoint(double);
	void MoveElevatorToSetPoint();

private:
	TalonSRX *elevatorMotor;
	DigitalInput *bottomSwitch;
	DigitalInput *topSwitch;
	double targetHeight;


};

} /* namespace Elevator */

#endif /* SRC_SUBSYSTEMS_ELEVATOR_H_ */
