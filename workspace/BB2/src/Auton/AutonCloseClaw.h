/*
 * AutonCloseClaw.h
 *
 *  Created on: Mar 29, 2018
 *      Author: Sabita Dhal
 */

#ifndef SRC_AUTON_AUTONCLOSECLAW_H_
#define SRC_AUTON_AUTONCLOSECLAW_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Intake.h>

class AutonCloseClaw: public AutoCommand {
public:
	AutonCloseClaw(Intake*);
	virtual ~AutonCloseClaw();

	void Initialize();
	bool Run();

private:
	Intake *intake;
};

#endif /* SRC_AUTON_AUTONCLOSECLAW_H_ */
