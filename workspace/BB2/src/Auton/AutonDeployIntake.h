/*
 * AutonDeployIntake.h
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#ifndef SRC_AUTON_AUTONDEPLOYINTAKE_H_
#define SRC_AUTON_AUTONDEPLOYINTAKE_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Intake.h>

class AutonDeployIntake : public AutoCommand {
public:
	AutonDeployIntake(Intake*);
	virtual ~AutonDeployIntake();

	void Initialize();
	bool Run();

private:
	Intake *intake;
};

#endif /* SRC_AUTON_AUTONDEPLOYINTAKE_H_ */
