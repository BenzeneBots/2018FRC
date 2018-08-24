/*
 * AutonStowIntake.h
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#ifndef SRC_AUTON_AUTONSTOWINTAKE_H_
#define SRC_AUTON_AUTONSTOWINTAKE_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Intake.h>
#include <WPILib.h>

class AutonStowIntake: public AutoCommand {
public:
	AutonStowIntake(Intake*);
	virtual ~AutonStowIntake();

	void Initialize();
	bool Run();

private:
	Intake *intake;
	Timer* autonTimer;
};

#endif /* SRC_AUTON_AUTONSTOWINTAKE_H_ */
