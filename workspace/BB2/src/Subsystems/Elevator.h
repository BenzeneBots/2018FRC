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

class Elevator {
public:
	Elevator(int);
	void SetEncoderPosition(int);
	double GetElevatorPosition();
	double GetElevatorRate();
	void SetToOutput(double);
	void SetElevatorTarget(double);
	void MoveElevator(double);

private:
	TalonSRX *elevatorMotor;
	int elevatorTargetPos;
	enum State {increasing, decreasing, joystick};
	State elevatorState;


};

class AutonElevator : public Elevator {

};

#endif /* SRC_SUBSYSTEMS_ELEVATOR_H_ */
