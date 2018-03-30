/*
 * AutonOpenClaw.h
 *
 *  Created on: Mar 29, 2018
 *      Author: Sabita Dhal
 */

#ifndef SRC_AUTON_AUTONOPENCLAW_H_
#define SRC_AUTON_AUTONOPENCLAW_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Intake.h>

class AutonOpenClaw: public AutoCommand {
public:
	AutonOpenClaw(Intake*);
	virtual ~AutonOpenClaw();

	void Initialize();
	bool Run();

private:
	Intake *intake;
};

#endif /* SRC_AUTON_AUTONOPENCLAW_H_ */
