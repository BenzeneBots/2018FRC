/*
 * AutonIntake.h
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#ifndef SRC_AUTON_AUTONINTAKE_H_
#define SRC_AUTON_AUTONINTAKE_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Intake.h>

class AutonIntake: public AutoCommand {
public:
	AutonIntake(Intake*, double);
	virtual ~AutonIntake();

	void Initialize();
	bool Run();

private:
	Intake* intake;
	Timer* intakeTimer;
	double timeoutTime;
};

#endif /* SRC_AUTON_AUTONINTAKE_H_ */
