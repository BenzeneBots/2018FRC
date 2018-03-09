/*
 * AutoDeployIntake.h
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#ifndef SRC_AUTON_AUTODEPLOYINTAKE_H_
#define SRC_AUTON_AUTODEPLOYINTAKE_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Intake.h>

class AutoDeployIntake : public AutoCommand {
public:
	AutoDeployIntake(Intake*);
	virtual ~AutoDeployIntake();

	void Initialize();
	bool Run();

private:
	Intake *intake;
};

#endif /* SRC_AUTON_AUTODEPLOYINTAKE_H_ */
