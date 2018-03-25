/*
 * AutonTurn.h
 *
 *  Created on: Mar 25, 2018
 *      Author: Murali
 */

#ifndef SRC_AUTON_AUTONTURN_H_
#define SRC_AUTON_AUTONTURN_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Drive.h>

class AutonTurn: public AutoCommand {
public:
	AutonTurn(Drive*, double);
	virtual ~AutonTurn();

	void Initialize();
	bool Run();

private:
	double targetAngle, leftSpeed, rightSpeed;
	Drive *drive;
	enum TurnState {turning, adjusting};
	TurnState turnState;
};

#endif /* SRC_AUTON_AUTONTURN_H_ */
