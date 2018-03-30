/*
 * AutonOuttake.h
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#ifndef SRC_AUTON_AUTONOUTTAKE_H_
#define SRC_AUTON_AUTONOUTTAKE_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Intake.h>
#include <WPILib.h>

class AutonOuttake: public AutoCommand {
public:
	AutonOuttake(Intake*, double);
	virtual ~AutonOuttake();

	void Initialize();
	bool Run();

private:
	Intake *intake;
	Timer* outtakeTimer;
	double timeoutTime;
};

#endif /* SRC_AUTON_AUTONOUTTAKE_H_ */
