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
	double GetElevatorHeight();
	void SetToOutput(double);
	bool SetElevatorTarget(double);
	bool MoveElevator(double);
	void SetJoystickControl();
	void MagicElevator(bool,bool,bool,double);
	void BenzeneElevator(bool,bool,bool,double);

private:
	TalonSRX *elevatorMotor;
	int elevatorTargetPos;
	enum State {increasing, decreasing, joystick};
	State elevatorState;
	bool lockElevator;

};

#endif /* SRC_SUBSYSTEMS_ELEVATOR_H_ */
