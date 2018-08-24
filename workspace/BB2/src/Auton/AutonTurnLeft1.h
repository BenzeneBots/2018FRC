/*
 * AutonTurnLeft1.h
 *
 *  Created on: Mar 30, 2018
 *      Author: Sabita Dhal
 */

#ifndef SRC_AUTON_AUTONTURNLEFT1_H_
#define SRC_AUTON_AUTONTURNLEFT1_H_

#include <Auton/AutoCommand.h>
#include <WPILib.h>
#include <Subsystems/Drive.h>

class AutonTurnLeft1: public AutoCommand {
public:
	AutonTurnLeft1(Drive*, double);
	virtual ~AutonTurnLeft1();

	void Initialize();
	bool Run();

private:
	Drive *drive;
	double targetAngle, rightSpeed, leftSpeed,speed;
	Timer* turnTimer;
	bool doneFlag;
};

#endif /* SRC_AUTON_AUTONTURNLEFT1_H_ */
