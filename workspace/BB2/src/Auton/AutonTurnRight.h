/*
 * AutonTurnRight.h
 *
 *  Created on: Mar 10, 2018
 *      Author: Murali
 */

#ifndef SRC_AUTON_AUTONTURNRIGHT_H_
#define SRC_AUTON_AUTONTURNRIGHT_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Drive.h>

class AutonTurnRight: public AutoCommand {
public:
	AutonTurnRight(Drive*, double);
	virtual ~AutonTurnRight();

	void Initialize();
	bool Run();

private:
	Drive *drive;
	double targetAngle, rightSpeed, leftSpeed;
	enum TurnState {turning, adjusting};
	TurnState turnState;

};

#endif /* SRC_AUTON_AUTONTURNRIGHT_H_ */
