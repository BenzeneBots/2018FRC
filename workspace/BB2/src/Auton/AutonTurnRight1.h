/*
 * AutonTurnRight1.h
 *
 *  Created on: Mar 30, 2018
 *      Author: Sabita Dhal
 */

#ifndef SRC_AUTON_AUTONTURNRIGHT1_H_
#define SRC_AUTON_AUTONTURNRIGHT1_H_

#include <Auton/AutoCommand.h>

#include <Auton/AutoCommand.h>
#include <Subsystems/Drive.h>

class AutonTurnRight1: public AutoCommand {
public:
	AutonTurnRight1(Drive*, double);
	virtual ~AutonTurnRight1();

	void Initialize();
	bool Run();

private:
	Drive *drive;
	double targetAngle, rightSpeed, leftSpeed,speed;
	Timer* turnTimer;
	bool doneFlag;

};

#endif /* SRC_AUTON_AUTONTURNRIGHT1_H_ */
