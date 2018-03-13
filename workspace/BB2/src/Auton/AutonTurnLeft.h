/*
 * AutonTurnLeft.h
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#ifndef SRC_AUTON_AUTONTURNLEFT_H_
#define SRC_AUTON_AUTONTURNLEFT_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Drive.h>

class AutonTurnLeft: public AutoCommand {
public:
	AutonTurnLeft(Drive*, double);
	virtual ~AutonTurnLeft();

	void Initialize();
	bool Run();

private:
	Drive *drive;
	double turnAngle, startAngle, targetAngle;
};

#endif /* SRC_AUTON_AUTONTURNLEFT_H_ */
