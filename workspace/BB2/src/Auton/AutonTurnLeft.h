/*
 * AutonTurnLeft.h
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#ifndef SRC_AUTON_AUTONTURNLEFT_H_
#define SRC_AUTON_AUTONTURNLEFT_H_

#include <Auton/AutoCommand.h>
#include <WPILib.h>
#include <Subsystems/Drive.h>

class AutonTurnLeft: public AutoCommand {
public:
	AutonTurnLeft(Drive*, double);
	virtual ~AutonTurnLeft();

	void Initialize();
	bool Run();

private:
	Drive *drive;
	double targetAngle, rightSpeed, leftSpeed;
	enum TurnState {turning, adjusting};
	TurnState turnState;
	double ScaleFunction(double);
	Timer* turnTimer;
};

#endif /* SRC_AUTON_AUTONTURNLEFT_H_ */
